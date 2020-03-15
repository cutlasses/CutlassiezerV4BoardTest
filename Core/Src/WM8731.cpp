#include "DebugOutput.h"
#include "main.h"
#include "WM8731.h"
#include "Util.h"

namespace
{

	const constexpr int WM8731_I2C_ADDR			= 0x1A;

	const constexpr int WM8731_REG_LLINEIN		= 0;
	const constexpr int WM8731_REG_RLINEIN		= 1;
	const constexpr int WM8731_REG_LHEADOUT		= 2;
	const constexpr int WM8731_REG_RHEADOUT		= 3;
	const constexpr int WM8731_REG_ANALOG		= 4;
	const constexpr int WM8731_REG_DIGITAL		= 5;
	const constexpr int WM8731_REG_POWERDOWN	= 6;
	const constexpr int WM8731_REG_INTERFACE	= 7;
	const constexpr int WM8731_REG_SAMPLING		= 8;
	const constexpr int WM8731_REG_ACTIVE		= 9;
	const constexpr int WM8731_REG_RESET		= 15;

	bool write_register_i2c( uint32_t reg, uint32_t val)
	{
		const constexpr int NUM_ATTEMPTS(16);

		for( int i = 0; i < NUM_ATTEMPTS; ++i )
		{
			uint8_t buffer[2];
			buffer[0] = (reg << 1) | ((val >> 8) & 1);
			buffer[1] = val & 0xFF;

			const HAL_StatusTypeDef status = HAL_I2C_Master_Transmit( &hi2c1, WM8731_I2C_ADDR << 1, buffer, 2, 100 );

			switch( status )
			{
				case HAL_OK:
				{
					DEBUG_TEXT("WM8731 I2C Success\n");
					return true;
				}
				case HAL_ERROR:
				{
					DEBUG_TEXT("WM8731 I2C failed - HAL_ERROR\n");
					break;
				}
				case HAL_BUSY:
				{
					DEBUG_TEXT("WM8731 I2C failed - HAL_BUSY\n");
					break;
				}
				case HAL_TIMEOUT:
				{
					DEBUG_TEXT("WM8731 I2C failed - HAL_TIMEOUT\n");
					break;
				}
			}
		}

		DEBUG_TEXT("WM8731 I2C failed - MAX ATTEMPTS REACHED\n");
		return false;
	}
}

namespace WM8731
{

	bool initialise()
	{
		if( !write_register_i2c( WM8731_REG_RESET, 0 ) )
		{
			return false; // WM8731 not responding
		}

		write_register_i2c( WM8731_REG_INTERFACE, 0x02 ); // I2S, 16 bit, MCLK slave
		write_register_i2c( WM8731_REG_SAMPLING, 0x20 );  // 256*Fs, 44.1 kHz, MCLK/1

		// In order to prevent pops, the DAC should first be soft-muted (DACMU),
		// the output should then be de-selected from the line and headphone output
		// (DACSEL), then the DAC powered down (DACPD).

		write_register_i2c( WM8731_REG_DIGITAL, 0x08 );   // DAC soft mute
		write_register_i2c( WM8731_REG_ANALOG, 0x00 );    // disable all

		write_register_i2c( WM8731_REG_POWERDOWN, 0x00 ); // codec powerdown

		write_register_i2c( WM8731_REG_LHEADOUT, 0x80 );  // volume off
		write_register_i2c( WM8731_REG_RHEADOUT, 0x80 );

		HAL_Delay(100); // how long to power up?

		write_register_i2c( WM8731_REG_ACTIVE, 1 );
		HAL_Delay(5);
		write_register_i2c( WM8731_REG_DIGITAL, 0x00 );   // DAC unmuted
		write_register_i2c( WM8731_REG_ANALOG, 0x10 );    // DAC selected

		return true;
	}

	bool input_select( INPUT_TYPE type )
	{
		if( type == INPUT_TYPE::LINE_IN )
		{
			return write_register_i2c( WM8731_REG_ANALOG, 0x12 );
		}
		else if( type == INPUT_TYPE::MIC )
		{
			return write_register_i2c( WM8731_REG_ANALOG, 0x15 );
		}

		return false;
	}

	bool set_input_gain( float gain )
	{
		// range is 0x00 (min) - 0x1F (max)
		const constexpr int max_gain(0x1F);

		const int scaled_gain = min_val( round_to_int<int>( gain * static_cast<float>(max_gain) ), max_gain );

		if( write_register_i2c( WM8731_REG_LLINEIN, scaled_gain ) )
		{
			if( write_register_i2c( WM8731_REG_RLINEIN, scaled_gain ) )
			{
				return true;
			}
		}

		return false;
	}

}
