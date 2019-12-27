/*
 * Termination.h
 *
 *  Created on: Aug 29, 2016
 *      Author: pourya
 */

//Guard
#ifndef TERMINATION_H_
#define TERMINATION_H_

//Headers
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include "GlobalDefinitions.h"

//Function prototypes
void terminationHandler(int signum);
// void atExitHandler(void);

#endif /* TERMINATION_H_ */
