#include "handlerbutton.h"
#include "useless_actions.h"
#include "servomoteur.h"

extern int max_arm;
extern int min_arm;
extern int max_lid;
extern int min_lid;
extern TIM_HandleTypeDef timerArm;
extern TIM_HandleTypeDef timerLid;


void close_box(){
	set_angle(100.0,&timerArm,TIM_CHANNEL_2, min_arm, max_arm);
	set_angle(100.0,&timerLid,TIM_CHANNEL_1, min_lid, max_lid);
	
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)){}
		
	set_angle(0.0,&timerArm,TIM_CHANNEL_2, min_arm, max_arm);
	set_angle(0.0,&timerLid,TIM_CHANNEL_1, min_lid, max_lid);
}

void handlerbutton(){	
			//random function choice & execution
			(Get_Random_Action())();
			
			//button put down
			close_box();
}
