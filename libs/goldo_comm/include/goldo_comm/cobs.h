#ifndef __GOLDO_COMM_COBS_H__
#define __GOLDO_COMM_COBS_H__
#include "goldo_comm/sink.h"


struct goldo_cobs_encoder_state
{
	struct goldo_comm_sink_ops* vtable;
	void* sink_ptr;
	uint8_t* read_ptr;
	uint8_t* write_ptr;
	uint8_t* code_ptr;
	/* Circular buffer for current encoding block*/
	uint8_t buffer[260];
};

/* Initialize COBS encoder state*/
void goldo_cobs_encoder_init(
	struct goldo_cobs_encoder_state* s,
	void* sink_ptr
	);

/* Encode data from in into out*/
size_t goldo_cobs_encoder_encode(
	struct goldo_cobs_encoder_state* s,
	const uint8_t* in_p,
	size_t in_size);

/* Read remaining data in buffer */
int goldo_cobs_encoder_flush(struct goldo_cobs_encoder_state* s);

/* End current packet. */
void goldo_cobs_encoder_end_packet(struct goldo_cobs_encoder_state* s);


struct goldo_cobs_decoder_state
{	
	struct goldo_comm_sink_ops* vtable;
	void* sink_ptr;
	uint8_t counter;
	/* Code of current encoding block*/
	uint8_t code; 
};

void goldo_cobs_decoder_init(
	struct goldo_cobs_decoder_state* s,
	void* sink_ptr
	);

/*! \brief Decode data.
*  \param[in] buffer Pointer to data to decode.
*  \param[in] size Size of the data to decode.
*  \return Number of bytes read from buffer.
*/
size_t goldo_cobs_decoder_decode(
	struct goldo_cobs_decoder_state* s,
	const uint8_t* buffer,
	size_t size);

#endif // __GOLDO_COMM_COBS_H__