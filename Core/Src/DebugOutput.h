#pragma once
#include "CompileSwitches.h"

void print_serial(const char* text);

inline bool _assert_fail( const char* assert, const char* msg )
{
	print_serial(assert);
	print_serial(" ");
	print_serial(msg);
	print_serial("\n");

	return true;
}

#ifdef DEBUG_OUTPUT
#define ASSERT_MSG(x, msg) ((void)((x) || (_assert_fail(#x,msg))))
#define DEBUG_TEXT(x)	print_serial(x);
#else
#define ASSERT_MSG(x, msg)
#define DEBUG_TEXT(x)
#endif

