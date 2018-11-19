#ifndef USELESSACTIONS_H_
#define USELESSACTIONS_H_

#include "stdint.h"

#define ACTION_TABLE_SIZE 100

typedef void (*FuncPtr)();

void Register_Action(FuncPtr fn, int weight);

FuncPtr Get_Random_Action();

#endif  /* !USELESSACTIONS_H_ */
