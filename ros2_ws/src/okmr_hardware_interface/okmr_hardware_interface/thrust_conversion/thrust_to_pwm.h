#pragma once
#include <string>

void loadThrustTable(double voltage);
void updateThrustTableIfVoltageChanged(double voltage);



double thrustToNormalizedPWM(double thrust);  // use if you want to input the normalized values between -1.0 and 1.0
int thrustToPWM(double thrust);               // use for 1100–1900 µs

