#ifndef TRANSBOT_EXAMPLE06_SEGWAY_CONFIG_H
#define TRANSBOT_EXAMPLE06_SEGWAY_CONFIG_H

#include <IMU.h>
#include <Filters.h>

#include "transbot_segway_motor_driver.h"

#define DEBUG

#define PWM_LIMIT                       885
#define CONTOL_PERIOD                   7000     // in microseconds

void startDynamixelControlInterrupt();
void imuInit();
void getAngle(float angle[3]);
void controlSegway();

#endif // TRANSBOT_EXAMPLE06_SEGWAY_CONFIG_H
