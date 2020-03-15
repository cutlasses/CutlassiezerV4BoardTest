#pragma once

#include <limits>
#include <type_traits>

/////////////////////////////////////////////////////

template <typename T>
T clamp( const T& value, const T& min, const T& max )
{
	if( value < min )
	{
		return min;
	}
	if( value > max )
	{
		return max;
	}
	return value;
}

template <typename T>
T max_val( const T& v1, const T& v2 )
{
	if( v1 > v2 )
	{
		return v1;
	}
	else
	{
		return v2;
	}
}

template <typename T>
T min_val( const T& v1, const T& v2 )
{
	if( v1 < v2 )
	{
		return v1;
	}
	else
	{
		return v2;
	}
}

template <typename T>
void swap( T& v1, T& v2 )
{
	T temp = v1;
	v1 = v2;
	v2 = temp;
}

/////////////////////////////////////////////////////

template <typename T>
T lerp( const T& v1, const T& v2, float t )
{
	return v1 + ( (v2 - v1) * t );
}

/////////////////////////////////////////////////////

inline constexpr int trunc_to_int( float v )
{
	return static_cast<int>( v );
}

template<typename T>
inline constexpr T round_to_int(float v)
{
	static_assert( std::is_integral<T>::value );
	if( v > 0.0f )
	{
		return static_cast<T>( v + 0.5f );
	}
	else
	{
		return static_cast<T>( v - 0.5f );
	}
}

/////////////////////////////////////////////////////

inline constexpr uint8_t reverse_byte( uint8_t byte_to_reverse )
{
	uint8_t b = byte_to_reverse;
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

/////////////////////////////////////////////////////

template < typename TYPE, int CAPACITY >
class RUNNING_AVERAGE
{
	TYPE                    m_values[ CAPACITY ];
	int                     m_current;
	int                     m_size;

public:

	RUNNING_AVERAGE() :
		m_values(),
		m_current(0),
		m_size(0)
{
}

	void add( TYPE value )
	{
		m_values[ m_current ] = value;
		m_current             = ( m_current + 1 ) % CAPACITY;
		++m_size;
		if( m_size > CAPACITY )
		{
			m_size              = CAPACITY;
		}
	}

	void reset()
	{
		m_size                = 0;
		m_current             = 0;
	}

	TYPE average() const
	{
		if( m_size == 0 )
		{
			return 0;
		}

		TYPE avg = 0;
		for( int x = 0; x < m_size; ++x )
		{
			avg += m_values[ x ];
		}

		return avg / m_size;
	}

	int size() const
	{
		return m_size;
	}
};

//// AUDIO ////

namespace DSP_UTILS
{

	template<typename SAMPLE_TYPE>
	SAMPLE_TYPE convert_unary_to_sample( float sample_unary )
	{
		ASSERT_MSG( sample_unary <= 1.0f && sample_unary >= -1.0f, "Sample is not unary" );
		if( sample_unary > 0.0f )
		{
			return round_to_int<SAMPLE_TYPE>( sample_unary * std::numeric_limits<SAMPLE_TYPE>::max() );
		}
		else
		{
			return round_to_int<SAMPLE_TYPE>( (-sample_unary) * std::numeric_limits<SAMPLE_TYPE>::min() );
		}
	}

	inline int16_t soft_clip_sample( int16_t sample, float clip_coefficient, bool debug = false )
	{
		// scale input sample to the range [-1,1]
		const float sample_f = static_cast<float>(sample) / static_cast<float>(std::numeric_limits<int16_t>::max());
		ASSERT_MSG( sample_f >= -1.01f && sample_f <= 1.01f, "Soft clip - sample scale error" );
		const float clipped_sample = sample_f - ( clip_coefficient * ( sample_f * sample_f * sample_f ) );

		// scale back to [int16 min, int16 max]
		const int16_t output_sample = round_to_int<int16_t>( clipped_sample * std::numeric_limits<int16_t>::max() );

		return output_sample;
	}

	// from http://polymathprogrammer.com/2008/09/29/linear-and-cubic-interpolation/
	inline float cubic_interpolation( float p0, float p1, float p2, float p3, float t )
	{
		const float one_minus_t = 1.0f - t;
		return ( one_minus_t * one_minus_t * one_minus_t * p0 ) + ( 3 * one_minus_t * one_minus_t * t * p1 ) + ( 3 * one_minus_t * t * t * p2 ) + ( t * t * t * p3 );
	}

	inline void fade_in_linear( int16_t* sample_buffer, int buffer_size )
	{
		const float fade_inc = 1.0f / buffer_size;
		float fade = 0.0f;
		for( int i = 0; i < buffer_size; ++i )
		{
			int16_t& sample = sample_buffer[i];
			sample = round_to_int<int16_t>( fade * sample );
			fade += fade_inc;
		}
	}

	inline void fade_out_linear( int16_t* sample_buffer, int buffer_size )
	{
		const float fade_dec = 1.0f / buffer_size;
		float fade = 1.0f;
		for( int i = 0; i < buffer_size; ++i )
		{
			int16_t& sample = sample_buffer[i];
			sample = round_to_int<int16_t>( fade * sample );
			fade -= fade_dec;
		}
	}

	inline int16_t read_sample_cubic( float read_head, const int16_t* sample_buffer, int buffer_size )
	{
		const int int_part      = trunc_to_int( read_head );
		const float frac_part   = read_head - int_part;

		auto read_sample = [sample_buffer, buffer_size]( int read_position ) -> int16_t
				{
			ASSERT_MSG( read_position >= 0 && read_position < buffer_size, "read_sample() OUT OF BOUNDS" );
			return sample_buffer[read_position];
				};

		float p0;
		if( int_part >= 2 )
		{
			p0            = read_sample( int_part - 2 );
		}
		else
		{
			// at the beginning of the buffer, assume previous sample was the same
			p0            = read_sample( 0 );
		}

		float p1;
		if( int_part <= 2 )
		{
			// reuse p0
			p1            = p0;
		}
		else
		{
			p1            = read_sample( int_part - 1 );
		}

		float p2        = read_sample( int_part );

		float p3;
		if( int_part < buffer_size - 1)
		{
			p3            = read_sample( int_part + 1 );
		}
		else
		{
			p3            = p2;
		}

		const float t   = lerp( 0.33333f, 0.66666f, frac_part );

		float sampf     = cubic_interpolation( p0, p1, p2, p3, t );
		return round_to_int<int16_t>( sampf );
	}

}
// DSP_UTILS
