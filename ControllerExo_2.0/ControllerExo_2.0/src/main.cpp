#include <Arduino.h>
#include <controller.h>

Controller ct;

void setup(){
    Serial.begin(9600);
}

void loop(){
    ct.checkModeController();
    Serial.print(ct.getMode());
    Serial.print(" ");
    Serial.println(ct.getDirection());
}