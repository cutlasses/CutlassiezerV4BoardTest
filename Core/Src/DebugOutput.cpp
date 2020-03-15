#include <string.h>

#include "DebugOutput.h"
#include "main.h"

void print_serial(const char* text)
{
	HAL_UART_Transmit( &huart6, (uint8_t*)text, strlen(text), HAL_MAX_DELAY );
}
