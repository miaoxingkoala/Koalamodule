#include "koalamodule.h"

koalamodule mymodule;
//koalamodule mymodule2;
//X位置：EN = 12, STEP = 26,DIR = 16,SW = 13
//Y位置：EN = 12, STEP = 25,DIR = 27,SW = 5
//Z位置：EN = 12, STEP = 17,DIR = 14,SW = 23
//丝杆模组stepspermm为400，同步带模组stepspermm为100
//丝杆模组行程为195，同步带模组行程为245

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  //void Koalamodule::attach(en,step,dir)
  mymodule.attach(12,25,27);
  mymodule.attachSwitch(5);
  mymodule.setStepspermm(400);
  //mymodule.invertdirection();
  mymodule.toOrigin();
  mymodule.setAcceleration(500);
  delay(1000);
  mymodule.toPosition(50,1500);
  delay(1000);
}

void loop() {
  while(digitalRead(5));
  delay(1000);
  mymodule.toOrigin();
  delay(200);
  mymodule.toPosition(125,1500);
  delay(200);
  mymodule.toPosition(75,1500);
  delay(200);
  mymodule.toPosition(195,1500);
  delay(200);
  mymodule.toPosition(95,1500);
  // delay(200);
  // mymodule.toPosition(120,2000);
  // delay(200);
  // mymodule.toPosition(140,2000);
  // delay(200);
  // mymodule.toPosition(160,2000);
  // delay(200);
  // mymodule.toPosition(195,2000);
  // delay(200);
}
