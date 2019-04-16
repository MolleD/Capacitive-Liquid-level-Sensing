#include "CapacitiveSensor.h"


CapacitiveSensor cs1 = CapacitiveSensor(1,6,2,3,4,5);


int state = cs1.get_SenseOneCycle();

void setup() {
Serial.begin(115200);
pinMode(3, OUTPUT);
}

void loop() {
  

long cs1v = cs1.capacitiveSensor(100);

Serial.println(cs1v);
delay(200);


}
