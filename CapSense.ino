#include "CapacitiveSensor.h"


CapacitiveSensor cs1 = CapacitiveSensor(1,2,3,4,5,6);


//int state = cs1.get_SenseOneCycle();

void setup() {
    cs1.set_CS_AutocaL_Millis(0xFFFFFFFF);
    Serial.begin(115200);
}

void loop() {
long start = millis();
long cs1v = cs1.capacitiveSensor(100);

Serial.print("Time: ");
Serial.println(millis() - start);
Serial.println(cs1v);
delay(250);


}
