#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
	extern C{
#endif
	
#include "stm32f10x.h"
#include <stdbool.h>
	
void systemInit(void);
bool systemTest(void);

void systemLaunch(void);

void systemStart(void);
void systemWaitStart(void);
void systemSetCanFly(bool val);
bool systemCanFly(void);

void systemLaunch(void);
	
#ifdef __cplusplus
	}
#endif
#endif
