#include "main.h"

int timer[5] = {0,0,0,0,0};

void startTimer(int slot){
   timer[slot] =  pros::millis();
}

int getTime(int slot){
    return (pros::millis() - timer[slot]);
}