#include "goldobot/tasks/gyro.hpp"
#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"
#include "FreeRTOS.h"
#include "stm32f3xx_hal.h"
#include "task.h"

#define CAL_TIME 100
#define SPI_FRAME_SZ 4
#define POOL_MAX_CNT 100000

using namespace goldobot;

extern "C"
{
	extern SPI_HandleTypeDef hspi1;
}

GyroTask::GyroTask() {
}

const char* GyroTask::name() const
{
	return "Gyro";
}

void GyroTask::taskFunction()
{
	unsigned long current_time = xTaskGetTickCount();
	unsigned long last_time = xTaskGetTickCount();
	float delta_time = 0;
	float float_angle = 0.0;
	float delta_angle = 0.0;
	int i = 0;
	angle = 0.0;
	rate = 0.0;
	bias = 0.0;

	//Read twice to clear the buffer
	goldo_gyro_get_id();
	part_id = goldo_gyro_get_id();

	//ID should start with 0x52, else, something went wrong !
	if((part_id >> 8) == 0x52){ //0x52 = 1010010
        init_status = 1;

        //Init temperature
        goldo_gyro_get_temperature();
        temperature = goldo_gyro_get_temperature();

        goldo_gyro_get_rate();
        while(i < CAL_TIME){
          rate = rate + goldo_gyro_get_rate();
          i ++;
          delay_periodic(100);
        }
        bias = rate / i;
    }
	while(1)
	{
		rate = goldo_gyro_get_rate() - bias;
		current_time = xTaskGetTickCount();
		delta_time = (current_time - last_time) / 1000.0;
		delta_angle = (rate * delta_time);
		angle += delta_angle;
		if(angle > 360.0){
			angle -= 360.0;
		}
		else if(angle < 0.0){
			angle += 360.0;
		}
		last_time = current_time;
		delay_periodic(50);
	}
}

int GyroTask::goldo_gyro_send_spi_frame(void) {
	HAL_GPIO_WritePin(NUCLEO_DYNA_DIR_GPIO_Port, NUCLEO_DYNA_DIR_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi1, spi_buf_out, spi_buf_in,SPI_FRAME_SZ);
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
	{

	}
	HAL_GPIO_WritePin(NUCLEO_DYNA_DIR_GPIO_Port, NUCLEO_DYNA_DIR_Pin, GPIO_PIN_SET);
	return 0;
}

int GyroTask::goldo_gyro_master_spi_read_word (unsigned int register_addr, unsigned int *pdata)
{
  int i;
  uint32_t val;
  int result;
  unsigned long  command       = 0;
  unsigned char  bitNo         = 0;
  unsigned char  sum           = 0;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  spi_buf_out[0] = ADXRS453_READ | (register_addr >> 7);
  spi_buf_out[1] = (register_addr << 1);
  // Parity bit
  command = ((unsigned long)spi_buf_out[0] << 24) |
            ((unsigned long)spi_buf_out[1] << 16) |
            ((unsigned short)spi_buf_out[2] << 8) |
			spi_buf_out[3];
  for(bitNo = 31; bitNo > 0; bitNo--){
      sum += ((command >> bitNo) & 0x1);
  }
  if(!(sum % 2)){
	  spi_buf_out[3] |= 1;
  }

  spi_buf_out[0] = char(command >> 24);
  spi_buf_out[1] = char(command >> 16);
  spi_buf_out[2] = char(command >> 8);

  result = goldo_gyro_send_spi_frame();
  if (result!=0) {
    return result;
  }

  val = ((unsigned short)spi_buf_in[1] << 11) |
        ((unsigned short)spi_buf_in[2] << 3) |
        (spi_buf_in[3] >> 5);

  *pdata = val;

  return 0;
}



int GyroTask::goldo_gyro_master_spi_write_word (unsigned int apb_addr, unsigned int data)
{
  int i;
  int result;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_gyro_send_spi_frame();
  if (result!=0) {
    return result;
  }

  /* 2) writing data */
  spi_buf_out[0] = 0x40;
  spi_buf_out[1] = (data>>24) & 0xff;
  spi_buf_out[2] = (data>>16) & 0xff;
  spi_buf_out[3] = (data>>8) & 0xff;
  spi_buf_out[4] = (data) & 0xff;
  spi_buf_out[5] = 0;

  result = goldo_gyro_send_spi_frame();
  if (result!=0) {
    return result;
  }

  return 0;
}

unsigned int GyroTask::goldo_gyro_get_id()
{
	unsigned int id = 0;
	goldo_gyro_master_spi_read_word(ADXRS453_REG_PID, &id);
	return id;
}

float GyroTask::goldo_gyro_get_temperature(){

    unsigned int register_value = 0;
    float          temperature   = 0;

    goldo_gyro_master_spi_read_word(ADXRS453_REG_TEM, &register_value);
    register_value = (register_value >> 6) - 0x31F;
    temperature = (float) register_value / 5;

    return temperature;
}

float GyroTask::goldo_gyro_get_rate(){
	unsigned int register_value = 0;
	float          rate         = 0.0;

	goldo_gyro_master_spi_read_word(ADXRS453_REG_RATE, &register_value);
	/*!< If data received is in positive degree range */
	if(register_value < 0x8000)
	{
	    rate = ((float)register_value / 80.0);
	}
	/*!< If data received is in negative degree range */
	else
	{
	    rate = (-1) * ((float)(0xFFFF - register_value + 1) / 80.0);
	}

	return rate;
}

float GyroTask::goldo_gyro_get_angle(){
	return angle;
}

float GyroTask::goldo_gyro_get_bias(){
	return bias;
}
