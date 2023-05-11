#include <positionalnew.h>
#include <Motor.h>
#include<EspNow.h>

Peer remote;
JSONVar feedback;

Motor rotationMotor(19,18); //motor
UniversalEncoder rotationEncoder(25,26 , 1); //encoder
positionalnew rMPID(&rotationMotor);

double AggKpRotation = 1.0, AggKiRotation = 0.0, AggKdRotation = 0;
double SoftKpRotation = 0.25, SoftKiRotation = 0.0, SoftKdRotation = 0;

int  rotatePulse, changePulse;
int rightLs = 35, leftLs = 32; //limit swtich
bool rLs, lLs, init_ = true;
int level = -1, resetPulse = 50000, finalPulse=2400;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(rightLs, INPUT_PULLUP);
  pinMode(leftLs, INPUT_PULLUP);

  rMPID.setThreshold(100);
  rotationMotor.setEncoder(&rotationEncoder);
  rMPID.setOutputLimits(-35, 35);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);


  setId("RotLS");
  remote.init("ReCON");
  remote.setOnRecieve(leftRotation, "Angll");//
  remote.setOnRecieve(rightRotation, "Anglr");//
  remote.setOnRecieve(resetAll, "rst");//
}

void loop() {
  rotatePulse = rotationMotor.getReadings();
  rLs = (bool)digitalRead(rightLs);
  lLs = (bool)digitalRead(leftLs);
  /*if (init_) {
    if (level != 3)
    {
      if (level == -1)
      {
        rMPID.setPulse(resetPulse);
        level = 0;
      }
      if (!rLs && level == 0)
      {
        rotationMotor.reset();
        rotationMotor.setReadings(1000);
        level = 1;
      }
      if (lLs && level == 1)
      {
        rMPID.setPulse(-resetPulse);
        level = 2;
      }
      if (!lLs && level == 2)
      {
        finalPulse = rotatePulse;
        Serial.println(finalPulse);
        rMPID.setPulse(finalPulse / 2);
        level = 3;
        init_ = !init_;
      }
    }
  }*/
  rMPID.compute();
}

void leftRotation(JSONVar msg)
{
  rMPID.setThreshold(100);
  rMPID.setOutputLimits(-60, 60);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  rMPID.setPulse(rotatePulse - (finalPulse / 4));
  Serial.println("rotate left ");
}
void rightRotation(JSONVar msg)
{
  rMPID.setThreshold(100);
  rMPID.setOutputLimits(-60, 60);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  rMPID.setPulse(rotatePulse + (finalPulse / 4));
  Serial.println("rotate right ");
}
void resetAll(JSONVar msg)
{
  init_ = true;
  Serial.println("rst ");
}
