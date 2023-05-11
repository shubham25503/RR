#include <EspNow.h>
#include <Motor.h>
#include <ESP32Servo.h>

Servo servotx, servorx;

Peer remote;

Peer rrShooter;

Motor leftMotor(4, 18);
Motor rightMotor(19, 21);
UniversalEncoder rightEncoder(25, 33, -1);
UniversalEncoder leftEncoder(26, 27, 1);

int Ls1 = 23, Ls2 = 13, pneumaticPin = 32;
int resetPulse = 50000, pulseDiff = 150, degreeDiff = 5;
int rxpin = 16, txpin = 17, degree, rakPWM = 0, pwmdiff = 5;
int degOffSet=10;
bool init_ = true, LS1, LS2, pnOC, limitstop = true, lastlimit = true;

double period;

void setup() {
  Serial.begin(115200);
  pinMode(32, OUTPUT);
  pinMode(Ls1, INPUT_PULLUP);
  pinMode(Ls2, INPUT_PULLUP);
  //  pinMode(leftLs1, INPUT_PULLUP);
  //  pinMode(leftLs2, INPUT_PULLUP);

  servotx.attach(txpin);
  servorx.attach(rxpin);
//  degree = 90;
//  servorx.write(degree);
//  servotx.write(180 - degree-degOffSet);
  Serial.println("servo1");

  leftMotor.invertDirection();
  leftMotor.setEncoder(&leftEncoder);
  rightMotor.setEncoder(&rightEncoder);

  setId("PiCSR");
  remote.init("ReCON");
  rrShooter.init("RsHTr");
  remote.setOnRecieve(rackLvl1, "Rlvl1");
  remote.setOnRecieve(rackLvl2, "Rlvl2");
  remote.setOnRecieve(servoLvl1, "Slvl1");
  remote.setOnRecieve(servoLvl2, "Slvl2");
  remote.setOnRecieve(rackSpeedUp, "Rexpp");
  remote.setOnRecieve(rackSpeedDown, "Rexpn");
  remote.setOnRecieve(servoExtraPDeg, "Sexdp");
  remote.setOnRecieve(servoExtraNDeg, "Sexdn");
  remote.setOnRecieve(pneumaticOC, "pnpicking");
  remote.setOnRecieve(servoSetting, "servoSet");
  remote.setOnRecieve(resetAll, "rst");
  remote.setOnRecieve(stopBot, "stopBot");
//  rrShooter.setOnRecieve(autoServo, "pnCls");
//  rrShooter.setOnRecieve(autoServo2, "pnOpn");
}
double leftPulse, rightPulse;
void loop() {
  LS1 = (bool)digitalRead(Ls1);
  LS2 = (bool)digitalRead(Ls2);
  if(Serial.available())
  {
    degree= Serial.readString().toInt();
    servorx.write(degree);
  servotx.write(180 - degree -degOffSet);
  Serial.println(String(degree)+" serial");
  }
//  if (init_) {
//    rightMotor.setPWM(-30);
//    leftMotor.setPWM(-30);
//  }
//  if (LS1 == 0 && LS2 == 0) {
//    if (limitstop)
//    {
//      rakPWM = 0;
//      leftMotor.setReadings(0);
//      rightMotor.setReadings(0);
//      rightMotor.setPWM(rakPWM);
//      leftMotor.setPWM(rakPWM);
//      init_ = false;
//      limitstop = false;
//      lastlimit = true;
//    }

//  }
//  leftPulse = leftMotor.getReadings();
//  rightPulse = rightMotor.getReadings();
//  //  Serial.println("left: " + String(leftPulse) + " right: " + String(rightPulse));
//  if (rightPulse >= 1150 && leftPulse >= 1150 && lastlimit)
//  {
//    rightMotor.setPWM(0);
//    leftMotor.setPWM(0);
//    lastlimit = false;
//  }
}
void rackLvl1(JSONVar msg)
{
  rakPWM = 30;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  Serial.println("rak1");
}
void rackLvl2(JSONVar msg)
{
  rakPWM = -30;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  Serial.println("rac2");
  limitstop = true;
}
void servoLvl1(JSONVar msg)
{
  degree = 90;
  servorx.write(degree);
  servotx.write(180 - degree -degOffSet);
  Serial.println(String(degree)+" servo1");
}
void servoLvl2(JSONVar msg)
{
  degree = 45;//40 to done
  servorx.write(degree);
  servotx.write(180 - degree -degOffSet);
  Serial.println(String(degree)+" servo2");
}
void rackSpeedUp(JSONVar msg)
{
  if (rakPWM < 0)
  {
    rakPWM = rakPWM - pwmdiff;
    rakPWM = rakPWM < -255 ? -255 : rakPWM;
    rightMotor.setPWM(rakPWM);
    leftMotor.setPWM(rakPWM);
  }
  if (rakPWM > 0)
  {
    rakPWM = rakPWM + pwmdiff;
    rakPWM = rakPWM > 255 ? 255 : rakPWM;
    rightMotor.setPWM(rakPWM);
    leftMotor.setPWM(rakPWM);
  }
  Serial.println("rack Speed up");
}
void rackSpeedDown(JSONVar msg)
{
  if (rakPWM < 0)
  {
    rakPWM = rakPWM + pwmdiff;
    rakPWM = rakPWM > 0 ? 0 : rakPWM;
    rightMotor.setPWM(rakPWM);
    leftMotor.setPWM(rakPWM);
  }
  if (rakPWM > 0)
  {
    rakPWM = rakPWM - pwmdiff;
    rakPWM = rakPWM < 0 ? 0 : rakPWM;
    rightMotor.setPWM(rakPWM);
    leftMotor.setPWM(rakPWM);
  }
  Serial.println("rack Speed down");
}

void servoExtraPDeg(JSONVar msg)
{
  degree = degree + degreeDiff;
  degree = degree > 180 ? 180 : degree;
  servorx.write(degree);
  servotx.write(180 - degree-degOffSet);
  Serial.println(String(degree) + " extra servo Pos");
}
void servoExtraNDeg(JSONVar msg)
{
  degree = degree - degreeDiff;
  degree = degree < 0 ? 0 : degree;
  servorx.write(degree);
  servotx.write(180 - degree-degOffSet);
  Serial.println(String(degree) + "extra servo Pos");
}
void pneumaticOC(JSONVar msg)
{
  if (pnOC == true) {
    Serial.println("Pneumatic Open");
    digitalWrite(32, HIGH);
    pnOC = false;
  }
  else if (pnOC == false) {
    Serial.println("Pneumatic Close");
    digitalWrite(32, LOW);
    pnOC = true;
  }
  Serial.println(JSON.stringify(msg));
}
void resetAll(JSONVar msg)
{
  //  digitalWrite(pneumaticPin, HIGH);
  rakPWM = 0;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  Serial.println("rst");
}
void servoSetting(JSONVar msg) {
  degree = 0;
  servorx.write(degree);
  servotx.write(180 - degree-degOffSet);
  Serial.println("servo2");
}
void stopBot(JSONVar msg)
{
  rightMotor.setPWM(0);
  leftMotor.setPWM(0);
}
/*
void autoServo(JSONVar msg)
{
  degree = degree - 15;
  servorx.write(degree);
  servotx.write(180 - degree);
  Serial.print("Degree "+String(degree));
}
void autoServo2(JSONVar msg)
{
  degree = degree + 15;
  servorx.write(degree);
  servotx.write(180 - degree);
  Serial.print("Degree "+String(degree));
}*/
