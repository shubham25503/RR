/*
    shooter left    shooter right
  rack levels
  left   /         /  right
       /         /
      /         /
  (leftLs2)        (rightLs2) level 2 (resetLevel)

    /       /
   /       /
  /       /
  (leftLs1)        (rightLs1) level 1



  Servo Levels
              left         right  level 2 (reset level) (45 degree)
                    /      /
                   /      /
                  /      /

  left  --------- level 1 (90 degree)
  right ---------
*/
#include <EspNow.h>
#include <Motor.h>
#include <positionalnew.h>
#include <ESP32Servo.h>

Servo servotx, servorx;

Peer remote;
JSONVar feedback;
Motor leftMotor(4, 18);
Motor rightMotor(19, 21);
UniversalEncoder rightEncoder(25, 33, -1);
UniversalEncoder leftEncoder(26, 27, 1);
//positionalnew rightMPID(&leftMotor);
//positionalnew leftMPID(&rightMotor);
//
//double AggKpLeft = 1.0, AggKiLeft = 0.0, AggKdLeft = 0;
//double SoftKpLeft = 0.75, SoftKiLeft = 0.0, SoftKdLeft = 0;
//double AggKpRight = 0.90, AggKiRight = 0.0, AggKdRight = 0;
//double SoftKpRight = 0.675, SoftKiRight = 0, SoftKdRight = 0;

int rightLs1 = 22, rightLs2 = 13, leftLs1 = 14, leftLs2 = 23, pneumaticPin = 32;
int resetPulse = 50000, pulseDiff = 150, degreeDiff = 5;
int rxpin = 16, txpin = 17, degree, rakPWM = 0, pwmdiff = 5;

//long rightPulse = 0, leftPulse = 0, rLvl2Pulse = 0, rLvl1Pulse = 1100;
//
//bool rLs1 = false, rLs2 = false, lLs1 = false, lLs2 = false;
//bool startLvl1 = false, startLvl2 = false, init_ = false;
//bool initUpperLimitRight = false, initUpperLimitLeft = false, intiLowerLimitLeft = false, intiLowerLimitRight = false;

void setup()
{
  Serial.begin(115200);
  pinMode(pneumaticPin, OUTPUT);
  pinMode(rightLs1, INPUT_PULLUP);
  pinMode(rightLs2, INPUT_PULLUP);
  pinMode(leftLs1, INPUT_PULLUP);
  pinMode(leftLs2, INPUT_PULLUP);

  servotx.attach(txpin);
  servorx.attach(rxpin);

  leftMotor.invertDirection();
  leftMotor.setEncoder(&leftEncoder);
  rightMotor.setEncoder(&rightEncoder);

  //  rightMPID.setThreshold(100);
  //  rightMPID.setOutputLimits(-40, 40);
  //  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  //  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);
  //
  //  leftMPID.setThreshold(100);
  //  leftMPID.setOutputLimits(-40, 40);
  //  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  //  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);

  setId("PiCSR");
  remote.init("ReCON");
  remote.setOnRecieve(rackLvl1, "Rlvl1");
  remote.setOnRecieve(rackLvl2, "Rlvl2");
  remote.setOnRecieve(servoLvl1, "Slvl1");
  remote.setOnRecieve(servoLvl2, "Slvl2");
  remote.setOnRecieve(rackExtraPPulse, "Rexpp");
  remote.setOnRecieve(rackExtraNPulse, "Rexpn");
  remote.setOnRecieve(servoExtraPDeg, "Sexdp");
  remote.setOnRecieve(servoExtraNDeg, "Sexdn");
  remote.setOnRecieve(pneumaticOpen, "pnopn");
  remote.setOnRecieve(pneumaticClose, "pncls");
  remote.setOnRecieve(resetAll, "rst");
}
void loop()
{
  if (Serial.available() > 0) {
    rakPWM = Serial.readStringUntil(',').toInt();
    degree = Serial.readStringUntil('\n').toInt();
    rightMotor.setPWM(rakPWM);
    leftMotor.setPWM(rakPWM);
    servorx.write(degree);
    servotx.write(190 - degree);
    Serial.println(String(rakPWM) + "," + String(degree));
  }

  /*

    leftPulse = leftMotor.getReadings();
    rightPulse = rightMotor.getReadings();
    Serial.println("left: " + String(leftPulse) + " right: " + String(rightPulse));

    rLs1 = !(bool)digitalRead(rightLs1);
    rLs2 = !(bool)digitalRead(rightLs2);
    lLs1 = !(bool)digitalRead(leftLs1);
    lLs2 = !(bool)digitalRead(leftLs2);
    if (!init_)
    {
    if (!lLs1 || !rLs1 && !startLvl1) // will go up initially to level 1
    {
      startLvl1 = true;
      leftMPID.setPulse(resetPulse);
      rightMPID.setPulse(resetPulse);
      Serial.println("Go to level 1");
    }

    if (lLs1 && !intiLowerLimitLeft) // left motor reached lower
    {
      Serial.println("left reached level 1");
      intiLowerLimitLeft = true;
      leftMPID.setPulse(leftPulse);
    }

    if (rLs1 && !intiLowerLimitRight) // right motor reached lower
    {
      Serial.println("right reached level 1");
      intiLowerLimitRight = true;
      leftMPID.setPulse(rightPulse);
    }

    if (intiLowerLimitLeft && intiLowerLimitRight && !startLvl2) // both motor reached level 1 and now goto level 2
    {
      leftMotor.reset();
      rightMotor.reset();
      leftMotor.setReadings(rLvl1Pulse);
      rightMotor.setReadings(rLvl1Pulse);
      leftMPID.setPulse(-resetPulse);
      rightMPID.setPulse(-resetPulse);
      Serial.println("Go to Level 2");
      startLvl2 = true;
    }

    if (lLs2 && st9artLvl2 && !initUpperLimitLeft) // left motor reached level 2
    {
      Serial.println("left reached level 1");
      initUpperLimitLeft = true;
      leftMPID.setPulse(leftPulse);
    }9

    if (rLs2 && startLvl2 && !initUpperLimitRight) // right motor reached level 2
    {
      Serial.println("left reached level 1");
      initUpperLimitRight = true;
      leftMPID.setPulse(leftPulse);
    }

    if (initUpperLimitLeft && initUpperLimitRight) // both reached level 2
    {
      rLvl2Pulse = (rightPulse + leftPulse) / 2;
      init_ = true;
    }
    }

    else if (init_)
    {

    }
    rightMPID.compute();
    leftMPID.compute();
  */
}

void rackLvl1(JSONVar msg)
{
  //  leftMPID.setPulse(rLvl1Pulse);
  //  rightMPID.setPulse(rLvl1Pulse);
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
  //  leftMPID.setPulse(rLvl2Pulse);
  //  rightMPID.setPulse(rLvl2Pulse);


  Serial.println("rac2");
}
void servoLvl1(JSONVar msg)
{
  degree = 90;
  servorx.write(degree);
  servotx.write(190 - degree);
  Serial.println("servo1");
}
void servoLvl2(JSONVar msg)
{
  degree = 45;
  servorx.write(degree);
  servotx.write(190 - degree);
  Serial.println("servo2");
}
void rackExtraNPulse(JSONVar msg)
{
  rakPWM = rakPWM - pwmdiff;
  rakPWM = rakPWM < -255 ? -255 : rakPWM;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  //  rightMPID.setThreshold(100);
  //  rightMPID.setOutputLimits(-40, 40);
  //  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  //  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);
  //  rightMPID.setPulse(rightPulse + pulseDiff);
  //
  //  leftMPID.setThreshold(100);
  //  leftMPID.setOutputLimits(-40, 40);
  //  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  //  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  //  leftMPID.setPulse(leftPulse + pulseDiff);
  Serial.println("extra rak pos");
}
void rackExtraPPulse(JSONVar msg)
{
  rakPWM = rakPWM + pwmdiff;
  rakPWM = rakPWM > 255 ? 255 : rakPWM;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  //  rightMPID.setThreshold(100);
  //  rightMPID.setOutputLimits(-40, 40);
  //  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  //  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);
  //  rightMPID.setPulse(rightPulse - pulseDiff);
  //
  //  leftMPID.setThreshold(100);
  //  leftMPID.setOutputLimits(-40, 40);
  //  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  //  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  //  leftMPID.setPulse(leftPulse - pulseDiff);

  Serial.println("extra rak neg");
}
void servoExtraPDeg(JSONVar msg)
{
  degree = degree + degreeDiff;
  degree = degree > 180 ? 180 : degree;
  servorx.write(degree);
  servotx.write(190 - degree);
  Serial.println("extra servo pos");
}
void servoExtraNDeg(JSONVar msg)
{
  degree = degree - degreeDiff;
  degree = degree < 0 ? 0 : degree;
  servorx.write(degree);
  servotx.write(190 - degree);
  Serial.println("extra servo neg");
}
void pneumaticClose(JSONVar msg)
{
  digitalWrite(pneumaticPin, HIGH);
  Serial.println("pn close");
}
void pneumaticOpen(JSONVar msg)
{
  digitalWrite(pneumaticPin, LOW);
  Serial.println("pn open");
}
void resetAll(JSONVar msg)
{
  digitalWrite(pneumaticPin, HIGH);
  //  init_= false;
  rakPWM = 0;
  rightMotor.setPWM(rakPWM);
  leftMotor.setPWM(rakPWM);
  Serial.println("rst");
}
