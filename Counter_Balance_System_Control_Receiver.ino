// PID motor position control.
#include <Wire.h>
#include <math.h>
float pi=M_PI;

#include <PinChangeInt.h>
#include <PID_v1.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10



#define endp 7
int ends;

double kp = 4 , ki = 1 , kd = 0.01;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;

int x_b=0;

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter   = '>';

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  Serial.begin (115200);// for debugging
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(endp,INPUT_PULLUP);
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);



  ends=digitalRead(endp);
  while(ends == 1)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    ends=digitalRead(endp);
    Serial.println(ends);
  }
  digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    delay (1000);
  encoderPos = 0;
  setpoint = 0;


  Wire.begin(1);                // join i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event
//  Wire.onRequest(requestEvent);
}


void loop() {
  x_b=constrain(x_b,2,250);
  setpoint = x_b*1133.0/(12.2*pi);                       // modify to fit motor and encoder characteristics, potmeter connected to A0

  input = encoderPos ;                                // data from encoder
  Serial.print(encoderPos);                      // monitor motor position
  Serial.print ("  \t");
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
  Serial.print(setpoint);
  Serial.print ("  \t");
  Serial.println(x_b);
  
  
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 0);
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));                        // drive motor CCW
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}


//Receive events
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
  //  Serial.print(c);         // print the character
  } 
  x_b = Wire.read();    // receive byte as an integer
 // Serial.println(x);        // print the integer
}

//void requestEvent() {
//  Wire.write(x_b); // respond with message of 8 bytes
//  // as expected by master
//}
