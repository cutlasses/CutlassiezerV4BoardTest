#include <memory.h>

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
const constexpr uint16_t HALF_TRANSMIT_BUFFER_SIZE(TRANSMIT_BUFFER_SIZE/2);

int16_t g_transmit_buffer[TRANSMIT_BUFFER_SIZE];
int16_t g_receive_buffer[RECEIVE_BUFFER_SIZE];

int16_t g_audio_buffer_left[AUDIO_BLOCK_SIZE];
int16_t g_audio_buffer_right[AUDIO_BLOCK_SIZE];
volatile bool in_interrupt = false;

void i2s_receive( const int16_t* receive_buffer, int16_t* audio_left, int16_t* audio_right, size_t size )
{
	// strip the left channel only
	size_t bi = 0;
	for( size_t i = 0; i < size; ++i )
	{
		audio_left[i]	= receive_buffer[bi++];
		audio_right[i]	= receive_buffer[bi++];
	}
}

void is2_transmit( int16_t* transmit_buffer, const int16_t* audio_left, const int16_t* audio_right, size_t size )
{
	// interleave left and right (currently left only)
	size_t bi = 0;
	for( size_t i = 0; i < size; ++i )
	{
		transmit_buffer[bi++]	= audio_left[i];
		transmit_buffer[bi++]	= audio_right[i];
	}
}

void audio_interrupt_update( int16_t buffer_index )
{
	ASSERT_MSG( !in_interrupt, "Receive interrupt interrupted?" );

	in_interrupt = true;

	// start reading/writing from the beginning
	i2s_receive( &(g_receive_buffer[buffer_index]), &(g_audio_buffer_left[0]), &(g_audio_buffer_right[0]), AUDIO_BLOCK_SIZE );

	// process audio

	is2_transmit( &(g_transmit_buffer[buffer_index]), &(g_audio_buffer_left[0]), &(g_audio_buffer_right[0]), AUDIO_BLOCK_SIZE );

	in_interrupt = false;
}

// TRANSMIT - transfer half complete
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start processing from the beginning
	audio_interrupt_update( 0 );
}

// TRANSMIT - transfer complete
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start processing from the midpoint
	audio_interrupt_update( HALF_TRANSMIT_BUFFER_SIZE );
}

/*
// RECEIVE - receive half complete
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	receive_complete = true;
}

// RECEIVE - receive complete
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	receive_complete = true;
}
*/

void initialise_sound_engine()
{
	DEBUG_TEXT("initialise_sound_engine()\n");

	// reset the transmit buffer
	memset( g_receive_buffer, 0, RECEIVE_BUFFER_SIZE );
	memset( g_transmit_buffer, 0, TRANSMIT_BUFFER_SIZE );
	memset( g_audio_buffer_left, 0, AUDIO_BLOCK_SIZE );
	memset( g_audio_buffer_right, 0, AUDIO_BLOCK_SIZE );


	// start off DMA
	if( HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint8_t*)g_transmit_buffer, TRANSMIT_BUFFER_SIZE ) )
	{
		DEBUG_TEXT("HAL_SAI_Transmit_DMA fail\r\n");
		//Error_Handler();
	}

	if (HAL_OK != HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)g_receive_buffer, RECEIVE_BUFFER_SIZE ) )
	{
		DEBUG_TEXT("HAL_SAI_Receive_DMA fail\r\n");
		//Error_Handler();
	}

	HAL_Delay(100);

	WM8731::initialise();
	WM8731::input_select(WM8731::INPUT_TYPE::LINE_IN);
	WM8731::set_input_gain(0.7f);

	DEBUG_TEXT("initialise_sound_engine() COMPLETED\n");
}

void sound_engine_loop()
{
	DEBUG_TEXT("Loop\n");

	HAL_Delay(1000);
}
