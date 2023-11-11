#include "MotorManager.hpp"

thor::Pins pins[10] = {{200, 2, 3}, {200, 4, 5}};
thor::MotorManager man(2);

void setup() 
{
    Serial.begin(9600);

    man.Setup(pins);
}

void loop() 
{
    man.Job();
}