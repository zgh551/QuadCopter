 #ifndef __CONSOLE_H
 #define __CONSOLE_H
 
 #ifdef __cplusplus
	extern C{
 #endif
	#include "stm32f10x.h"
	#include <stdbool.h>	
/**
 * Initialize the console
 */
void consoleInit(void);

bool consoleTest(void);

/**
 * Put a character to the console buffer
 *
 * @param ch character that shall be printed
 * @return The character casted to unsigned int or EOF in case of error
 */
int consolePutchar(int ch);

/**
 * Put a null-terminated string on the console buffer
 *
 * @param str Null terminated string
 * @return a nonnegative number on success, or EOF on error. 
 */
int consolePuts(char *str);

/**
 * Flush the console buffer
 */
void consoleFlush(void);

/**
 * Macro implementing consolePrintf with eprintf
 * 
 * @param FMT String format
 * @patam ... Parameters to print
 */
#define consolePrintf(FMT, ...) eprintf(consolePutchar, FMT, ## __VA_ARGS__)

 #ifdef __cplusplus
	}
 #endif
 #endif
