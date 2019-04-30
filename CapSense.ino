#include "CapacitiveSensor.h"


CapacitiveSensor cs1 = CapacitiveSensor(1,2,3,4,5,6);


//int state = cs1.get_SenseOneCycle();

void setup() {
Serial.begin(115200);
}

void loop() {
  

long cs1v = cs1.capacitiveSensor(100);

Serial.println(cs1v);
delay(15);


}
