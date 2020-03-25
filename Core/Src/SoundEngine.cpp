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

int16_t transmit_buffer[TRANSMIT_BUFFER_SIZE];
int16_t receive_buffer[RECEIVE_BUFFER_SIZE];

volatile int16_t audio_buffer_left[AUDIO_BLOCK_SIZE];
volatile bool receive_complete = false;

void audio_stream_receive( const int16_t* buffer, size_t size )
{
	receive_complete = false;

	// strip the left channel only
	for( size_t i = 0; i < size; i+=2 )
	{
		audio_buffer_left[i] = buffer[i];
	}

	// process audio

	receive_complete = true;
}

void audio_stream_transmit( int16_t* buffer, size_t size )
{
	// interleave left and right (currently left only)
	ASSERT_MSG( receive_complete, "Receive interrupt interrupted?" );

	for( size_t i = 0; i < size; i+=2 )
	{
		buffer[i] = audio_buffer_left[i];
		buffer[i+1] = 0;
	}
}

// TRANSMIT - transfer half complete
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start writing from the beginning
	audio_stream_transmit( &(transmit_buffer[0]), HALF_TRANSMIT_BUFFER_SIZE );
}

// TRANSMIT - transfer complete
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	// start writing from midpoint
	audio_stream_transmit( &(transmit_buffer[HALF_TRANSMIT_BUFFER_SIZE]), HALF_TRANSMIT_BUFFER_SIZE );

}

// RECEIVE - receive half complete
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audio_stream_receive( &(receive_buffer[0]), HALF_TRANSMIT_BUFFER_SIZE );
}

// RECEIVE - receive complete
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	audio_stream_receive( &(receive_buffer[HALF_TRANSMIT_BUFFER_SIZE]), HALF_TRANSMIT_BUFFER_SIZE );
}

void initialise_sound_engine()
{
	DEBUG_TEXT("initialise_sound_engine()\n");

	// reset the transmit buffer
	memset( transmit_buffer, 0, TRANSMIT_BUFFER_SIZE );
	memset( const_cast<int16_t*>(audio_buffer_left), 0, AUDIO_BLOCK_SIZE );


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
