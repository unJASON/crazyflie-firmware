/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.  
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "pos_localization.h"

#include "commander.h"

#define DEBUG_MODULE "HELLOWORLD"

static setpoint_t setpoint;

void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy,float vz, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity.z = vz;
  setpoint->velocity_body = true;
}


//take off for better performance
void takeoff(float height){
  float tmp = 0.0f;
  while (true)
  {
    if (tmp+0.4f>=height){
      break;
    }else{
      tmp = tmp + 0.4f;
      setHoverSetpoint(&setpoint,0.0,0.0,0.0,0.2,tmp);
      commanderSetSetpoint(&setpoint,3);
      vTaskDelay(M2T(200));
    }
  }
  
  setHoverSetpoint(&setpoint,0.0,0.0,0.0,0.2,height);
  commanderSetSetpoint(&setpoint,3);
  vTaskDelay(M2T(200));
}


void land(){
  
}
void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  // initiate EKF
  uint8_t addr=initLocalization();
  // float height = 0.4f;
  // fly path
  // stage 1 : converge stage
  vTaskDelay(M2T(5000));
  if (addr == 1){
    
    // takeoff(height);
    // start the task after take off
    xTaskCreate(localizationTask,"Localization",3*configMINIMAL_STACK_SIZE, NULL,ZRANGER_TASK_PRI,NULL );
    // for(int i=0;i<1000;i++){
    //   setHoverSetpoint(&setpoint,0.0f,0.0f,0.2f,height,0.0f);
    //   commanderSetSetpoint(&setpoint,3);
    //   vTaskDelay(M2T(200));
    // }
    // fly repeatedly
    
    // DEBUG_PRINT("%d\n",addr);
  }else{
    // do nothiong
    xTaskCreate(localizationTask,"Localization",3*configMINIMAL_STACK_SIZE, NULL,ZRANGER_TASK_PRI,NULL );
  }
    
}

