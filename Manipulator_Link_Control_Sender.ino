#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

//Remote control
#define cp1 12
#define cp2 13
#define cp3 7
#define cp4 8
int ch1, ch2, ch3, ch4;
int chlow = 1300;
int chhigh = 1700;

//Servos
#include <Servo.h>
#define sp1 9
#define sp2 10
#define sp3 11
Servo Servo1;
Servo Servo2;
Servo Servo3;

const int MPU_addr1 = 0x68;
float xa, ya, za, roll, pitch;

float xm_l1=0, ym_l1=0, x_l1=0, y_l1=0, m_b=0, m_l1=0, l1=0, cog_l1=0;                                                            //Link 1
float xm_l2=0, ym_l2=0, x_l2=0, y_l2=0, m_l2=0, l2=0, cog_l2=0;                                                                   //Link 2
float x_p_temp=0, y_p_temp=0, x_p=0, y_p=0, m_p=0, x_b=0;                                                                         //Payload
float th1=0, theta1=0, th1_low=0, th1_high=0, th1_temp=0, th1s=0, ph1=0;                                                          //Theta 1
float th2=0, theta2=0, th2_low=0, th2_high=0, th2_temp=0, th2s=0, th2_abs=0, th2_abs_temp=0, th2_low_a=0, th2_high_a=0, ph2=0;    //Theta 2
float th3=0, theta3=0, th3_low=0, th3_high=0, th3_temp=0, th3s=0, th3_abs=0, th3_abs_temp=0;                                      //Theta 3
float link_spd=0.3*M_PI/180.00, spd=1*M_PI/180.00, n1=0, n2=0, n=1, n_max=0, res1=1, res2=1, e1=0, e2=0;//try 2/3 and 6/7?
byte x_send;                                                                                                                      //I2C Send value

const unsigned long waiting = 6000;
unsigned long PT1 = 0;
int point;


void InitializePins()
{
  //Servos 
  Servo1.attach(sp1);
  Servo2.attach(sp2);
  Servo3.attach(sp3);
  
  //Remote control input 
}

void Constants()
{
  //Angle limitations
  th1_low=-120*M_PI/180.00;
  th1_high=-10*M_PI/180.00;
  th2_low=0;
  th2_high=140*M_PI/180.00;
  th2_low_a= -90*M_PI/180.00;
  th2_high_a= M_PI*6/7.00+th2_low_a;
  th3_low=-90*M_PI/180;
  th3_high=90*M_PI/180;
  //Masses
  m_b = 0.545;
  m_l1 = 0.053;
  m_l2 = 0.035;
  m_p = 0.2;

  //Link lengths
  l1 = 0.15;
  l2 = 0.14;
  
  //COG locations
  cog_l1=0.08;
  cog_l2=0.082;

  //Starting position
  th1s = th1_high-75*M_PI/180.00;
  th2s = -th1s; //Starts th2_abs horizontal
  th3s = 0;
  
  th1 = th1s;
  th2 = th2s;
  th3 = th3s;
  th2_abs = th1 + th2;
  th3_abs = th1 + th2 + th3;
  
  th1_temp = th1;
  th2_temp = th2;
  th3_temp = th3;
  th2_abs_temp = th2_abs;
  th3_abs_temp = th3_abs;
}

void RemoteControl()
{

  if (ch1 <= chlow)
  {th1_temp = th1-spd*2.00/3.00;}
  else if (ch1 >= chhigh)
  {th1_temp = th1+spd*2.00/3.00;}
  else
  {th1_temp = th1;}
  if (ch2 <= chlow)
  {th2_temp = th2+spd*6.00/7.00;}
  else if (ch2 >= chhigh)
  {th2_temp = th2-spd*6.00/7.00;}
  else
  {th2_temp = th2;}
  if (ch3 <= chlow)
  {th3_temp = th3+4*spd;}
  else if (ch3 >= chhigh)
  {th3_temp = th3-4*spd;}
  else
  {th3_temp = th3;}

  
  th2_abs_temp = th1_temp+th2_temp;
  th3_abs_temp = th1_temp+th2_temp+th3_temp;
  x_p_temp = l1 * cos (th1_temp) + l2 * cos (th2_abs_temp);
  y_p_temp = l1 * sin (th1_temp) + l2 * sin (th2_abs_temp);
  if (x_p_temp>0 && y_p_temp<0)
  {
    th1=th1_temp;
    th2=th2_temp;
    th3=th3_temp;
  }
  else
  {
    th1_temp=th1;
    th2_temp=th2;
    th3_temp=th3;
  }
}

void ConstrainLinks()
{
  th1=constrain(th1,th1_low,th1_high);
  th2=constrain(th2,th2_low,th2_high);
  th2_abs=th1+th2;
  th2_abs=constrain(th2_abs,th2_low_a,th2_high_a);
  th3=constrain(th3,th3_low,th3_high);
  th3_abs=th1+th2+th3;
  th3_abs=constrain(th3_abs,th3_low,th3_high);
}

void Link2Servo()
{
  theta1=((th1)-th1_low)*(180/M_PI)*3.00/2.00;
  theta2=((th2_abs)*180/M_PI+90)*7.00/6.00;
  theta3=((th3_abs)*180/M_PI+90);
}

void ConstrainServos()
{
  theta1=constrain(theta1,0,180);
  theta2=constrain(theta2,0,180);
  theta3=constrain(theta3,0,180);
}

void WriteServos()
{
  Servo1.write(theta1);
  Servo2.write(theta2);
  Servo3.write(theta3);
}

void ForwardKinematics()
{
  xm_l1 = cog_l1 * cos (th1);
  xm_l2 = l1 * cos (th1) + cog_l2 * cos (th2_abs);
  ym_l1 = cog_l1 * sin (th1);
  ym_l2 = l1 * sin (th1) + cog_l2 * sin (th2_abs);

  x_l1 = l1 * cos (th1);
  x_l2 = l1 * cos (th1) + l2 * cos (th2_abs);
  y_l1 = l1 * sin (th1);
  y_l2 = l1 * sin (th1) + l2 * sin (th2_abs);

  x_p = l1 * cos (th1) + l2 * cos (th2_abs);
  y_p = l1 * sin (th1) + l2 * sin (th2_abs);
}

void BatteryLocation()
{
  x_b = -(m_l1 * xm_l1 + m_l2 * xm_l2 + m_p * x_p) / m_b;
}

void SendValue()
{
  x_send=abs(x_b*1000);
  Wire.beginTransmission(1); // transmit to device #1
  Wire.write(x_send);              // sends float
  Wire.endTransmission();    // stop transmitting
}

void ReceiveValue()
{
//  Wire.requestFrom(1, 8);    // request from slave device #1 8 bytes 
//  while (Wire.available()) { // slave may send less than requested
//    char c = Wire.read(); // receive a byte as character
//    Serial.print(c);         // print the character
//  }
//  Serial.println();
}

void PrintValues()
{
//  Serial.print("Ch1:"); 
//  Serial.print(ch1);
  Serial.print (millis());
  Serial.print("\t"); 
  Serial.print(theta1,1);
//  Serial.print("\t Ch2:");
//  Serial.print(ch2);
  Serial.print("\t"); 
  Serial.print(theta2,1);
//  Serial.print("\t Ch3:");
//  Serial.print(ch3);
  Serial.print("\t");
  Serial.print(theta3,1);
//  Serial.print("\t Ch4:");
//  Serial.print(ch4);
  Serial.print("\t");
  Serial.print("X_p: ");
  Serial.print("\t");
  Serial.print(x_p);
  Serial.print("\t"); 
  Serial.print("Y_p: ");
  Serial.print("\t");
  Serial.print(y_p);
  Serial.print("\t"); 
  Serial.print("X_b: ");
  Serial.print("\t");
  Serial.print(x_b);
  Serial.print("\t"); 
//  Serial.print("roll = ");
//  Serial.print("\t");
//  Serial.print(roll);
//  Serial.print("\t"); 
  Serial.print("pitch = ");
  Serial.print("\t");
  Serial.println(pitch);
}

void InverseKinematics(float x0, float y0)
{
  //Inverse Kinematics
  ph2=(acos((pow(x0,2)+pow(y0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)));
  ph1=(atan((y0/x0))-atan((l2*sin(ph2))/(l1+l2*cos(ph2))));
}

void ErrorCheck()
{
  e1 = abs (th1 - ph1);
  e2 = abs (th2 - ph2);
  //Angle restrictions (hardware limitations)
  if (th1 >= th1_low && th1 <= th1_high)
  {
    if (y_p <= 0 && e1 >= (res1 * M_PI / 180.00))
    {
      if (n1 <= n2)
      {
        th1 = th1 + (n1 / n2) * (link_spd) * (ph1 - th1) / abs (th1 - ph1);
      }
      else
      {
        th1 = th1 + (link_spd) * (ph1 - th1) / abs (th1 - ph1);
      }
    }
  }
  if (th2 >= th2_low && th2 <= th2_high)
  {
    if (y_p <= 0 && e2 >= (res2 * M_PI / 180.00))
    {
      if (n2 <= n1)
      {
        th2 = th2 + (n2 / n1) * (link_spd) * (ph2 - th2) / abs (th2 - ph2);
      }
      else
      {
        th2 = th2 + (link_spd) * (ph2 - th2) / abs (th2 - ph2);
      }
    }
  }
}

void PlannedMotion(float x0, float y0)
{
  InverseKinematics(x0,y0);
  //Number of iterations, sets motor speed
  n1 = (abs (th1 - ph1)) / (res1 * M_PI / 180.00);
  n2 = (abs (th2 - ph2)) / (res2 * M_PI / 180.00);
  n=1;
  if (n1>n2){n_max=n1;}
  else {n_max=n2;}
  while (n<=n_max+1)
  {
    InverseKinematics(x0,y0);
    ErrorCheck();
    ConstrainLinks();
    Link2Servo();
    ConstrainServos();
    ForwardKinematics();
    BatteryLocation();
    SendValue();
    WriteServos();
    n++;
    PrintValues();
  }
  
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  Wire.begin(); //Initializes i2c transfer
  pinMode(cp1, INPUT); 
  pinMode(cp2, INPUT);
  pinMode(cp3, INPUT);
  pinMode(cp4, INPUT);
  InitializePins();
  Constants();
  Link2Servo();
  ConstrainServos();
  WriteServos();
  PrintValues();
  Wire.beginTransmission(MPU_addr1);                 //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);                        //end the transmission
  delay (1000);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  unsigned long CT = millis();
  ch1 = pulseIn(cp1, HIGH); 
  ch2 = pulseIn(cp2, HIGH);
  ch3 = pulseIn(cp3, HIGH);
  ch4 = pulseIn(cp4, HIGH);
  if (ch4>=chhigh)
  {
    RemoteControl();
    ConstrainLinks();
    Link2Servo();
    ConstrainServos();
    ForwardKinematics();
    BatteryLocation();
    SendValue();
    WriteServos();
    ReceiveValue();
    PrintValues();
    PT1 = CT;
  }
  if (ch4<=chlow)
  {
    if (CT - PT1 >= waiting)
    {
      point++;
      PT1 = CT;
    }
    switch (point)
    {
    case 1:
    PlannedMotion(0,-0.29);
    break;
    case 2:
    PlannedMotion(0.1,-0.1);
    break;
    case 3:
    PlannedMotion(0.14,-0.15);
    break;
    case 4:
    PlannedMotion(0.285,-0.02);
    break;
    case 5:
    PlannedMotion(0.14,-0.15);
    break;
    case 6:
    PlannedMotion(0.1,-0.1);
    break;
    default:
    point = 0;
    break;
    }
  }
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 6, true); //get six bytes accelerometer data

  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();

  pitch = atan2(ya , za) * 180.0 / PI;
  roll = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;
}
