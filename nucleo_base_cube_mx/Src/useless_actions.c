#include "stdlib.h"
#include "useless_actions.h"
#include "servomoteur.h"


extern int max_arm;
extern int min_arm;
extern int max_lid;
extern int min_lid;
extern TIM_HandleTypeDef timerArm;
extern TIM_HandleTypeDef timerLid;

typedef void (*FuncPtr)();

int action_table_ind = 0;
FuncPtr action_table[ACTION_TABLE_SIZE];

void Register_Action(FuncPtr fn, int weight) {
	for (int i = 0; i < weight; ++i) {
		if (action_table_ind < ACTION_TABLE_SIZE) {
			action_table[action_table_ind++] = fn;
		}
	}
}

FuncPtr Get_Random_Action() {
	return action_table[rand() % action_table_ind];
}



