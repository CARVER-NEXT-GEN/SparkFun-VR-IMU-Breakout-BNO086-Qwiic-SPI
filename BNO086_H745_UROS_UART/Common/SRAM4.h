/*
 * SRAM4.h
 *
 *  Created on: Nov 6, 2024
 *      Author: tim
 */

#ifndef SRAM4_H_
#define SRAM4_H_
#include "main.h"
#define SHARED_MEMORY_ADDRESS 0x38000000 //SRAM4 address

typedef struct{
	uint32_t State1;
	double Data[30]; //Out of Complier scope So can bigger sa it not more than 64kB which not notice us

}SharedMemory;

volatile SharedMemory *const SRAM4 = (SharedMemory*)(SHARED_MEMORY_ADDRESS); //Pointer to SharedMemory

extern volatile SharedMemory *const SRAM4;
#endif /* SRAM4_H_ */
