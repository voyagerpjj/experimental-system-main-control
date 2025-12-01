#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include <math.h>
#include "motor_control.h"

typedef struct{
    uint16_t target_speed[MOTOR_COUNT];
}   target_speed_t;
void control_init(void);

void control_runing(void);

#endif // CONTROL_H
