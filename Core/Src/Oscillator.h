#pragma once

#include <cmath>

#include "Util.h"

template< typename SAMPLE_TYPE, uint16_t SAMPLE_RATE >
class Oscillator
{
	float			m_num_samples_in_cycle			= 0.0f;
	uint32_t		m_current_sample				= 0;

public:

	void			set_frequency( float frequency_hz )
	{
		// round to integer for simplicity
		m_num_samples_in_cycle = SAMPLE_RATE / frequency_hz;
	}

	SAMPLE_TYPE		next_sample()
	{
		const constexpr float two_pi	= M_PI * 2.0f;
		const float r					= (m_current_sample * two_pi) / m_num_samples_in_cycle; // in the range 0..2pi
		const float sample				= sinf( r );

		++m_current_sample;
		if( m_current_sample >= truncf(m_num_samples_in_cycle) )
		{
			m_current_sample = 0;
		}

		return DSP_UTILS::convert_unary_to_sample<SAMPLE_TYPE>( sample );
	}

	// TODO add modes other than sin
};
