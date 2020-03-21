#include "DebugOutput.h"
#include "main.h"
#include "Oscillator.h"
#include "SoundEngine.h"
#include "WM8731.h"

const constexpr uint16_t SAMPLE_RATE(44100);
const constexpr uint16_t AUDIO_BLOCK_SIZE(128);
const constexpr uint16_t NUM_TRANSMIT_BLOCKS(2);	// double buffered
const constexpr uint16_t NUM_CHANNELS(2);			// left and right
const constexpr uint16_t TRANSMIT_BUFFER_SIZE(AUDIO_BLOCK_SIZE*NUM_TRANSMIT_BLOCKS*2);
const constexpr uint16_t RECEIVE_BUFFER_SIZE(AUDIO_BLOCK_SIZE*NUM_TRANSMIT_BLOCKS*2);

int16_t transmit_buffer[TRANSMIT_BUFFER_SIZE];
int16_t receive_buffer[RECEIVE_BUFFER_SIZE];

Oscillator<int16_t, SAMPLE_RATE> sine_osc;

// TRANSMIT - transfer half complete
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start writing from the beginning
	const int16_t start_write_block = 0;
	const int16_t end_write_block = TRANSMIT_BUFFER_SIZE / 2;

	for( int16_t si = start_write_block; si < end_write_block; si+=2 )
	{
		const int16_t sample	= sine_osc.next_sample();
		transmit_buffer[si]		= sample;
		transmit_buffer[si+1]	= sample;
	}
}

// TRANSMIT - transfer complete
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start writing from midpoint
	const int16_t start_write_block = TRANSMIT_BUFFER_SIZE / 2;
	const int16_t end_write_block = TRANSMIT_BUFFER_SIZE;

	for( int16_t si = start_write_block; si < end_write_block; si+=2 )
	{
		const int16_t sample	= sine_osc.next_sample();
		transmit_buffer[si]		= sample;
		transmit_buffer[si+1]	= sample;
	}
}

// RECEIVE - receive half complete
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{

}

// RECEIVE - receive complete
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{

}

void initialise_sound_engine()
{
	DEBUG_TEXT("initialise_sound_engine()\n");

	// fill transmit buffer with sine wave
	/*
	const constexpr float buffer_time = (1.0f / SAMPLE_RATE) * AUDIO_BLOCK_SIZE * 2;
	const constexpr float buffer_freq = 1.0f / buffer_time;
	sine_osc.set_frequency(buffer_freq);

	for( int16_t si = 0; si < TRANSMIT_BUFFER_SIZE; si+=2 )
	{
		const int16_t sample	= sine_osc.next_sample();
		transmit_buffer[si]		= sample;
		transmit_buffer[si+1]	= sample;
	}
	*/


	// reset the transmit buffer
	for( int16_t& sample : transmit_buffer )
	{
		sample = 0;
	}

	sine_osc.set_frequency(441.0f);

	// start off DMA
	if( HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint8_t*)transmit_buffer, TRANSMIT_BUFFER_SIZE ) )
	{
		DEBUG_TEXT("HAL_SAI_Transmit_DMA fail\r\n");
		//Error_Handler();
	}

	if (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)receive_buffer, RECEIVE_BUFFER_SIZE ) )
	{
		DEBUG_TEXT("HAL_SAI_Receive_DMA fail\r\n");
		//Error_Handler();
	}

	HAL_Delay(100);

	WM8731::initialise();

	DEBUG_TEXT("initialise_sound_engine() COMPLETED\n");
}

void sound_engine_loop()
{
	DEBUG_TEXT("Loop\n");

	HAL_Delay(1000);
}
