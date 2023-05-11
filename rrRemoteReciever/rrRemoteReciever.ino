#include <Arduino_JSON.h>
#include <EspNow.h>
#include <remoteSerial.h>
Peer baseDirection;
Peer rrRingPick;
Peer rrShooter;
Peer rrRotation;
JSONVar serialData;
JSONVar pickData;
JSONVar shooterData;


bool switchMode = false, ignore = false;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(921600, SERIAL_8N1, 16, 17);
  registerSerial(&Serial2);
  setId("ReCON");
  baseDirection.init("RagNR");
  rrRingPick.init("PiCSR");
  rrShooter.init("RsHTr");
  //  rrRotation.init("RotLS");
  Serial2.setTimeout(1);

  baseDirection.setOnRecieve(baseDefaultHandler);
  rrRingPick.setOnRecieve(pickDefaultHandler);

  setBaseReciever(sendBase, "base");

  setSerialReciever(cross, "cro");
  setSerialReciever(circle, "cir");
  setSerialReciever(triangle, "tri");
  setSerialReciever(square, "squ");

  setSerialReciever(up, "up");
  setSerialReciever(down, "dow");
  setSerialReciever(right, "rig");
  setSerialReciever(left, "lef");

  setSerialReciever(ps, "ps");
  setSerialReciever(touchPad, "tpad");
  setSerialReciever(share, "share");
  setSerialReciever(option, "opt");

  setSerialReciever(rOne, "rOne");
  setSerialReciever(lOne, "lOne");
}

void loop()
{
  serialListener();
}

void sendBase(JSONVar msg)
{
  baseDirection.send(msg);
  //  Serial.println(msg);
}

void ps(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "rst";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
  else if (switchMode)
  {
    shooterData["type"] = "rst";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
    rrRotation.send(shooterData);
  }
}

void touchPad(String msg)
{
  shooterData["type"] = "stopBot";
  pickData["type"] = "stopBot";
  rrRingPick.send(pickData);
  rrShooter.send(shooterData);
  baseDirection.send(shooterData);
}

void cross(String msg)
{
  if (!switchMode)
  {
    if (!ignore)
    {
      ignore = true;
      Serial.println("ignore");
    }
    else if (ignore)
    {
      pickData["type"] = "Rlvl1";
      Serial.println(JSON.stringify(pickData));
      rrRingPick.send(pickData);
    }
  }
  else if (switchMode)
  {
    shooterData["type"] = "Pole1";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
  }
}
void circle(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "Slvl2";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
  else if (switchMode)
  {
    shooterData["type"] = "Pole2";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
  }
}

void triangle(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "Rlvl2";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
  else if (switchMode)
  {
    shooterData["type"] = "Pole3";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
  }
}

void square(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "Slvl1";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
}

void up(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "Rexpp";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
  else if (switchMode)
  {
    shooterData["type"] = "bldcinc";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
  }
}

void down(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "Rexpn";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
  else if (switchMode)
  {
    shooterData["type"] = "bldcdec";
    Serial.println(JSON.stringify(shooterData));
    rrShooter.send(shooterData);
  }
}

void right(String msg)
{
  //  if (!switchMode)
  //  {
  pickData["type"] = "Sexdp";
  Serial.println(JSON.stringify(pickData));
  rrRingPick.send(pickData);
  //  }
  //  else if (switchMode)
  //  {
  //    shooterData["type"] = "Anglr";
  //    Serial.println(JSON.stringify(shooterData));
  //    rrRotation.send(shooterData);
  //  }
}

void left(String msg)
{
  //  if (!switchMode)
  //  {
  pickData["type"] = "Sexdn";
  Serial.println(JSON.stringify(pickData));
  rrRingPick.send(pickData);
  //  }
  //  else if (switchMode)
  //  {
  //    shooterData["type"] = "Angll";
  //    Serial.println(JSON.stringify(shooterData));
  //    rrRotation.send(shooterData);
  //  }
}

void share(String msg)
{
  if (!switchMode)
  {
    pickData["type"] = "servoSet";
    Serial.println(JSON.stringify(pickData));
    rrRingPick.send(pickData);
  }
}

void option(String msg)
{
  Serial.println("SwitchMode- " + String(switchMode));
  switchMode = !switchMode;
}

void rOne(String msg)
{


  shooterData["type"] = "pnshooter";
  rrShooter.send(shooterData);
  Serial.println(shooterData);
}

void lOne(String msg)
{
  pickData["type"] = "pnpicking";
  rrRingPick.send(pickData);
  Serial.println(pickData);
}

void baseDefaultHandler(JSONVar msg)
{
}

void pickDefaultHandler(JSONVar msg)
{
}
