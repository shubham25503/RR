//1 - -
//2 - 1
//3 - 1800
#include <ESP32Servo.h>
#include<EspNow.h>

Servo leftBLDC, rightBLDC;
int leftbldc = 15 , rightbldc = 22; //bldc

Peer remote;
Peer rrPicking;
Peer extRR;
JSONVar pnClose, Shooter;
int currentSpeed = 1000, speedDiff = 10;
int change = 2;
int pneumaticPin = 32;
bool pnOC = false;
void setup() {
  Serial.begin(115200);

  pinMode(32, OUTPUT);

  leftBLDC.attach(leftbldc);
  rightBLDC.attach(rightbldc);
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);



  setId("RsHTr");
  remote.init("ReCON");
  rrPicking.init("PiCSR");
  extRR.init("EtrRR");
  remote.setOnRecieve(poleOne, "Pole1");//
  remote.setOnRecieve(poleTwo, "Pole2");//
  remote.setOnRecieve(poleThree, "Pole3");//
  remote.setOnRecieve(pneumaticOC, "pnshooter");//
  //  remote.setOnRecieve(pneumaticClose, "pncls");//
  remote.setOnRecieve(incSpeed, "bldcinc");//
  remote.setOnRecieve(decSpeed, "bldcdec");//
  remote.setOnRecieve(resetAll, "rst");//
  remote.setOnRecieve(stopBot, "stopBot");//
  remote.setOnRecieve(changeSD, "chngbldc");
}

void loop() {
  if (Serial.available())
  {
    currentSpeed = Serial.readString().toInt();
    leftBLDC.write(currentSpeed);
    rightBLDC.write(currentSpeed);
    Serial.println("CurrentSpeed: " + String(currentSpeed));
  }
}
void poleOne(JSONVar msg)
{
  currentSpeed = 1250;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  Serial.println(JSON.stringify(msg));
  Serial.println("CurrentSpeed: " + String(currentSpeed));
  sendData();
}
void poleTwo(JSONVar msg)
{
  currentSpeed = 1350;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  Serial.println(JSON.stringify(msg));
  Serial.println("CurrentSpeed: " + String(currentSpeed));
  sendData();
}
void poleThree(JSONVar msg)
{
  currentSpeed = 1500;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  Serial.println(JSON.stringify(msg));
  Serial.println("CurrentSpeed: " + String(currentSpeed));
  sendData();
}
void pneumaticOC(JSONVar msg)
{
  if (pnOC == true) {
    Serial.println("Pneumatic Open");
    digitalWrite(32, HIGH);
    pnOC = false;
    //    pnClose["type"] = "pnOpn";
        delay(500);
    //    rrPicking.send(pnClose);
  }
  else if (pnOC == false) {
    Serial.println("Pneumatic Close");
    digitalWrite(32, LOW);
    pnOC = true;
    //    pnClose["type"] = "pnCls";
        delay(500);
    //    rrPicking.send(pnClose);
  }
  Serial.println(JSON.stringify(msg));
}
void incSpeed(JSONVar msg)
{
  currentSpeed = currentSpeed + speedDiff;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  Serial.println(JSON.stringify(msg));
  Serial.println("CurrentSpeed: " + String(currentSpeed));
  sendData();
}
void decSpeed(JSONVar msg)
{
  currentSpeed = currentSpeed - speedDiff;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  Serial.println(JSON.stringify(msg));
  Serial.println("CurrentSpeed: " + String(currentSpeed));
  sendData();
}
void resetAll(JSONVar msg)
{
  currentSpeed = 1000;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  //  digitalWrite(pneumaticPin, HIGH);
  sendData();
}
void stopBot(JSONVar msg)
{
  currentSpeed = 1000;
  leftBLDC.write(currentSpeed);
  rightBLDC.write(currentSpeed);
  sendData();
}
void changeSD(JSONVar msg)
{
  if (change == 1)
  {
    speedDiff = 10;
    change = 2;
  }
  else if (change == 2)
  {
    speedDiff = 5;
    change = 1;
  }
}
void sendData()
{
  Shooter["speed"]=currentSpeed;
  Shooter["type"]= "shtrdt";
  extRR.send(Shooter);
}
