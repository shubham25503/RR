#include <EspNow.h>
Peer baserr;
#include <RoteryBase.h>
RoteryBase r1;
Direction remoteData;
Motor m1(27,14);//left front
Motor m2(32,33);//right front
Motor m3(22,21);//right rear
Motor m4(25,26);//left rear

void setup()
{
  Serial.begin(115200);
  r1.setMotors(&m1, &m2, &m3, &m4);
  r1.setup();
  setId("RagNR");
  baserr.init("ReCON");
  //  m1.invertDirection();
  //  m2.invertDirection();
  //  m3.invertDirection();M
  baserr.setOnRecieve(baseDirection, "drive");
  baserr.setOnRecieve(stopBot, "stopBot");
}

void loop() {
  if (Serial.available() > 0)
  {
    r1.getUserInRef()->input();
  }

  r1.compute();
}
void baseDirection(JSONVar msg)
{
  r1.getUserInRef()->parseJson(msg);
}
void stopBot(JSONVar msg)
{
  m1.setPWM(0);
  m2.setPWM(0);
  m3.setPWM(0);
  m4.setPWM(0);
}
