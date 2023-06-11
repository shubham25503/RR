#include <PS4-ActionMonitor.h>
String datastring;
int basex, basey, baser;
int baselX, baselY, baserX, baserY;
long currentTime = 0;
static long previousTime;
bool rumble = false, done = true, opt = false, init_ = false, zeroCheck = false, pressed = false;
bool autoDown = false, autoCompute = false, autoBase = false;
bool setAllAnalog = false , prevclr=true;
int  color = 2, fixedPWM, outPutX, outPutY ;

void setup()
{

  Serial.begin(115200);
  Serial2.begin(921600);
  initPS4("1e:22:22:22:22:11");
//    initPS4("08:00:27:EC:10:61");
  //  initPS4("22:22:22:22:22:22");

  getAllAnalog(setAllAnalog);
  //Attaching function with buttons

  square->attachPress(sqaurePressed);
  square->attachRelease(sqaureReleased);

  circle->attachPress(circlePressed);
  circle->attachRelease(circleReleased);

  triangle->attachPress(trianglePressed);
  triangle->attachRelease(triangleReleased);

  cross->attachPress(crossPressed);
  cross->attachRelease(crossReleased);

  up->attachPress(upPressed);
  up->attachRelease(upReleased);

  down->attachPress(downPressed);
  down->attachRelease(downReleased);

  left->attachPress(leftPressed);
  left->attachRelease(leftReleased);

  right->attachPress(rightPressed);
  right->attachRelease(rightReleased);

  l1->attachPress(l1Pressed);
  l1->attachRelease(l1Released);

  r1->attachPress(r1Pressed);
  r1->attachRelease(r1Released);


  rJoystick->attachAnalogX(rightjoystickX);
  rJoystick->attachAnalogY(rightjoystickY);

  share->attachPress(sharePressed);
  share->attachRelease(shareReleased);

  options->attachPress(optionPressed);
  options->attachRelease(optionReleased);

  psButton->attachPress(psPressed);
  psButton->attachRelease(psReleased);

  touchpad->attachPress(touchpadPressed);
  touchpad->attachRelease(touchpadReleased);

  leftJoystick->attachAllData(base);
  //  allAnalog->attachAllAnalogData(directionalBase);
}
int lx, ly, rightX, rightY;
void loop()
{
    if (color == 1)
    {
      PS4.setLed(255, 0, 0);
    }
    else if (color == 2)
    {
      PS4.setLed(0, 255, 0);
    }
    else if (color ==3)
    {
      PS4.setLed(255,20,147);
    }
    PS4.sendToController();
  delay(10);
    //Serial.println(PS4.Battery());
  
}
void rightjoystickX(int v)
{
  outPutX = v;
  if (outPutX > 30) {
    fixedPWM = 50;
  }
  else if (outPutX < -30) {
    fixedPWM = -50;
  }
  else
  {
    fixedPWM = 0;
  }
  rightX = fixedPWM;
  if (abs(outPutX) >= abs(outPutY))
  {
    datastring = String(rightX) + "," + "0" + "," + "0" + "," + "\n";
    Serial2.print(datastring);
  }
  else if (abs(outPutX) < abs(outPutY)) {
    datastring = "0," + String(rightY) + ",0," + "\n";
    Serial2.print(datastring);
  }
  //  rightX = map(rightX, 0, 125, 0, 50);
  //  if (abs(rightX) >= abs(rightY))
  //  {
  //    datastring = String(rightX) + "," + "0" + "," + "0" + "," + "\n";
  //    Serial2.print(datastring);
  //  }
  //  else if (abs(rightX) < abs(rightY))
  //  {
  //    datastring = "0," + String(rightY) + ",0," + "\n";
  //    Serial2.print(datastring);
  //  }
  //  Serial.println("RightJoystick="+String(rightX)+","+String(rightY));
  //  Serial.println(datastring);
}
void rightjoystickY(int v)
{
  outPutY = v;
  if (outPutY > 30) {
    fixedPWM = 50;
  }
  else if (outPutY < -30) {
    fixedPWM = -50;
  }
  else
  {
    fixedPWM = 0;
  }
  rightY =  fixedPWM;
  if (abs(outPutX) >= abs(outPutY))
  {
    datastring = String(rightX) + "," + "0" + "," + "0" + "," + "\n";
    Serial2.print(datastring);
  }
  else if (abs(outPutX) < abs(outPutY)) {
    datastring = "0," + String(rightY) + ",0," + "\n";
    Serial2.print(datastring);
  }
  //  rightY = map(rightY, 0, 125, 0, 50);
  //  if (abs(rightX) >= abs(rightY))
  //  {
  //    datastring = String(rightX) + "," + "0" + "," + "0" + "," + "\n";
  //    Serial2.print(datastring);
  //  }
  //  else if (abs(rightX) < abs(rightY))
  //  {
  //    datastring = "0," + String(rightY) + ",0," + "\n";
  //    Serial2.print(datastring);
  //  }
  //  Serial.println("RightJoystick="+String(rightX)+","+String(rightY));
  //  Serial.println(datastring);
}
void l1Pressed()
{
  PS4.setRumble(0, 225);
  Serial2.print("lOne\n");
//  Serial.print("l1 Pressed\n");
}
void l1Released()
{
  PS4.setRumble(0, 0);
//  Serial.print("l1 Released\n");
}

void r1Pressed()
{
  PS4.setRumble(225, 0);
  Serial2.print("rOne\n");
//  Serial.print("r1 Pressed\n");
}
void r1Released()
{
  PS4.setRumble(0, 0);
//  Serial.println("r1 Released");
}

void crossPressed()
{
  Serial2.print("cro\n");
//  Serial.print("crossPressed\n");
}
void crossReleased()
{
//  Serial.print("crossReleased\n");
}

void circlePressed()
{
  Serial2.print("cir\n");
//  Serial.print("circlePressed\n");
}
void circleReleased()
{
//  Serial.print("circlePressed\n");
}

void trianglePressed()
{
  Serial2.print("tri\n");
//  Serial.print("trianglePressed\n");
}
void triangleReleased()
{
//  Serial.print("triangleReleased\n");
}

void sqaurePressed()
{
  Serial2.print("squ\n");
//  Serial.print("sqaure pressed\n");
}
void sqaureReleased()
{
//  Serial.print("sqaure Released\n");
}

void upPressed()
{
  Serial2.print("up\n");
//  Serial.print("up Pressed\n");
}
void upReleased()
{
  // Serial2.print("rel\n");
//  Serial.print("up Released\n");
}

void downPressed()
{
  Serial2.print("dow\n");
//  Serial.print("down Pressed\n");
}
void downReleased()
{
  // Serial2.print("rel\n");
//  Serial.print("down Released\n");
}

void rightPressed()
{
  Serial2.print("rig\n");
//  Serial.println("right");
}
void rightReleased()
{
//  Serial.print("rightReleased\n");
}

void leftPressed()
{
  Serial2.print("lef\n");
//  Serial.print("left Pressed\n");
}
void leftReleased()
{
  // Serial2.print("rel\n");
//  Serial.print("left Released\n");
}

void sharePressed()
{
  Serial2.print("share\n");
  if(color == 1)
  {
    color=3;
  }
  else if(color == 3)
  {
    color=1;
  }
//  Serial.print("share Pressed\n");
}
void shareReleased()
{
//  Serial.print("share Released\n");
}

void optionPressed()
{
  Serial2.print("opt\n");
//  Serial.print("option pressed\n");
  PS4.setRumble(225, 225);
  if (color == 1 || color == 3)
  {
    if(color==1)
    {
      prevclr=true;
    }
    else
    {
      prevclr= false;
    }
    color = 2;
  }
  else if (color == 2)
  {
    if(prevclr)
    {
      color=1;
    }
    else
    {
      color=3;
    }
  }
}
void optionReleased()
{
  PS4.setRumble(0, 0);
//  Serial.print("option Released\n");
}

void psPressed()
{
  Serial2.print("ps\n");
//  Serial.print("ps\n");
}
void psReleased()
{
//  Serial.print("ps Released\n");
}

void touchpadPressed()
{
  Serial2.print("tpad\n");
//  Serial.print("touchpad pressed\n");
}
void touchpadReleased()
{
//  Serial.print("touchpad Released\n");
}
void base(int x, int y, int r)
{
  basex = x;
  basey = y;
  baser = r;
  datastring = String(basex) + "," + String(basey) + "," + String(baser) + "," + "\n";
  Serial2.print(datastring);
  //  Serial.println(datastring);
  //  Serial2.print(JSON.stringify(baseData)+"\n");
}

void directionalBase(int lX, int lY, int rX, int rY, int r)
{
  baselX = lX;
  baselY = lY;
  baserX = rX;
  baserX = rX;
  baser = r;
  datastring = String(baselX) + "," + String(baselY) + "," + String(baserX) + "," + String(baserY) + "," + String(baser) + "," + "\n";
  //  Serial.println("lX: " + String(lX) + " lY: " + String(lY) + " rX: " + String(rX) + " rY: " + String(rY) + " , " + String(r));
}
