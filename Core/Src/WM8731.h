#pragma once

namespace WM8731
{
	enum class INPUT_TYPE
	{
		LINE_IN,
		MIC
	};

	bool	initialise();

	bool	input_select( INPUT_TYPE type );
	bool	set_input_gain( float gain );
}
