#include "stm32f3xx_hal.h"
extern "C"
{
	#include "stm32f3xx_hal_tim.h"
}


namespace goldobot { namespace platform {

struct PwmChannel
{
	uint8_t timer_id{0xff};
	uint8_t channel_id{0xff};
	uint8_t sign_port{0xff};
	uint8_t sign_pin{0xff};
};

struct DeviceEncoder
{
	uint8_t timer_id{0xff};
	uint8_t flags;

};

extern PwmChannel g_pwm_channels[8];
extern DeviceEncoder g_encoders[4];
extern TIM_HandleTypeDef g_tim_handles[11];

void hal_timer_init(const DeviceConfigTimer* config);
void hal_pwm_init(const DeviceConfigPwm* config);
void hal_encoder_init(const DeviceConfigEncoder* config);

}} // namespace goldobot::platform
