#include <stdio.h>
#include "Motor.h"
#include "Main.h"
#include "hmc5883l/hmc5883l.h"  //Digital Compass

uint16_t ultim_proporcional_compass=0;


void PID_compass_following(int direction){ //0 forward,1 backwards
  int speed_M_esquerre=0;
  int speed_M_dret=0;
  int proporcional_compass= get_heading_hmc5883l()-compass_direction_to_follow;
  int derivatiu_compass = proporcional_compass-ultim_proporcional_compass;
  ultim_proporcional_compass = proporcional_compass;

  //int power_difference = proporcional_compass*10  + derivatiu_compass*2/3;
  int power_difference = proporcional_compass*2/3;
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.

  speed_M_esquerre=velocitat-power_difference;
  speed_M_dret=velocitat+power_difference;

  if (direction==0){//FORWARD
    Motor_left_forward(speed_M_esquerre<255?speed_M_esquerre:255);
    Motor_right_forward(speed_M_dret<255?speed_M_dret:255);
  }else{
    Motor_right_reverse(speed_M_esquerre<255?speed_M_esquerre:255);
    Motor_left_reverse(speed_M_dret<255?speed_M_dret:255);
  }
}
