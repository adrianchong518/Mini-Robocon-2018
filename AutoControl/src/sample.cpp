#include <Arduino.h>
#include <Servo.h>
Servo myservo;

//const int upperSensorpin = 8;
//const int lowerSensorpin = 7;
//int upperSensorRead;
//int lowerSensorRead;

const int upperMotorpin = 2;
const int lowerMotorpin = 3;

int upperIsWhite = 0;
int upperRotation = 0;

int lowerIsWhite = 0;
int lowerRotation = 0;

int upperSpeed = 85; //在此調整發射台轉數之參數
int lowerSpeed = 60;

unsigned long previousMillis_a = 0;
unsigned long previousMillis_b = 0;
unsigned long currentMillis = 0;

int interval_1_forward = 1000;

int interval_2_backward = 5000;
int interval_2_north_east = 3000;

int interval_3_west = 2000;
int interval_3_forward = 3000;

int interval_4_backward = 3000;
int interval_4_south_east = 6000;
int interval_4_forward = 500;

int interval_5_backward = 500;
int interval_5_north_west = 6000;
int interval_5_forward = 3000;

long start_time;
long duration = millis() - start_time;

const int _m101 = 10, _m102 = 11, _m201 = 8, _m202 = 9, _m301 = 7, _m302 = 6, //For teensymotor's motions pins
    _m401 = 5, _m402 = 4;
double _m1_os = 1, _m2_os = 1, _m3_os = 1, _m4_os = 1; //For teensymotor's offsets
int _power = 100, _power2 = 255, _power3 = 60;         //Normal speed and faster speed

//int stepper_dir=17,stepper_pul=18,stepper_top=15,stepper_bottom=16;     //For stepper's pins
int steps = 6400, seconds = 500;
int a1 = 22, a2 = 23, b1 = 24, b2 = 25;
int stepper_top = 42, stepper_bottom = 43;

int servo_pin = 12; //For servo's pin
int servo_upper = 160, servo_lower = 45, servo_define_sec = 1000;

//bool bool_a;
//bool bool_b;

//Normal speed
void forward()
{ //1 //Motor move forward .
  analogWrite(_m101, _power * _m1_os);
  digitalWrite(_m102, LOW);
  analogWrite(_m201, _power * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power * _m3_os);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power * _m4_os);
}

void backward()
{ //2 //Motor move backward .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power * _m1_os);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power * _m2_os);
  analogWrite(_m301, _power * _m3_os);
  digitalWrite(_m302, LOW);
  analogWrite(_m401, _power * _m4_os);
  digitalWrite(_m402, LOW);
}

void left()
{ //3 //Motor turn left .
  analogWrite(_m101, _power * _m1_os);
  digitalWrite(_m102, LOW);
  analogWrite(_m201, _power * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void right()
{ //4 //Motor turn right .
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power * _m3_os);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power * _m4_os);
}

void north_west()
{ //5 //Motor move to NW .
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  analogWrite(_m201, _power * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power * _m3_os);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void south_west()
{ //6 //Motor move to SW .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power * _m1_os);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  analogWrite(_m401, _power * _m4_os);
  digitalWrite(_m402, LOW);
}

void north_east()
{ //7 //Motor move to NE .
  analogWrite(_m101, _power * _m1_os);
  digitalWrite(_m102, LOW);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power * _m4_os);
}

void north_east_soft()
{ //7 //Motor move to NE .
  analogWrite(_m101, _power3 * _m1_os);
  digitalWrite(_m102, LOW);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power3 * _m4_os);
}

void south_east()
{ //8 //Motor move to SE .
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power * _m2_os);
  analogWrite(_m301, _power * _m3_os);
  digitalWrite(_m302, LOW);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void west()
{ //9 //Motor move to west .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power * _m1_os);
  analogWrite(_m201, _power * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power * _m3_os);
  analogWrite(_m401, _power * _m4_os);
  digitalWrite(_m402, LOW);
}

void east()
{ //10 //Motor move to east
  analogWrite(_m101, _power * _m1_os);
  digitalWrite(_m102, LOW);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power * _m2_os);
  analogWrite(_m301, _power * _m3_os);
  digitalWrite(_m302, LOW);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power * _m4_os);
}

void stop_all()
{ //11 //Motor stop moving
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void stepper_upward()
{ //12 //Control stepper(upward)

  for (int qqq = 0; qqq < 10000; qqq++)
  {
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    if (digitalRead(stepper_top) == HIGH)
    {
      break;
    } 
  }
}

void stepper_upward_high()
{ //12 //Control stepper(upward)

  for (int qqqq = 0; qqqq < 100000; qqqq++)
  {
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    if (digitalRead(stepper_top) == HIGH)
    {
      break;
    }
  }
}

void stepper_downward()
{ //13 //Control stepper(downward)
  for (int www = 0; www < 10000; www++)
  {
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    if (digitalRead(stepper_bottom) == HIGH)
    {
      break;
    }
  }
}

void stepper_downward_high()
{ //13 //Control stepper(downward)
  for (int wwww = 0; wwww < 100000; wwww++)
  {
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    if (digitalRead(stepper_bottom) == HIGH)
    {
      break;
    }
  }
}

void servo_move()
{ //14 //Servo move up and down
  myservo.write(servo_upper);
  delay(servo_define_sec);
  myservo.write(servo_lower);
  //delay(20);
}

char value;

void shotting_ball_2_wheels()
{ //15 //Control speed of 2 shotting ball wheels
  if ((value == 'r' && upperSpeed < 255) || (value == 'R' && upperSpeed < 255))
  {
    upperSpeed++;
  }
  else if ((value == 'f' && upperSpeed > 0) || (value == 'F' && upperSpeed > 0))
  {
    upperSpeed--;
  }
  else if ((value == 't' && lowerSpeed < 255) || (value == 'T' && lowerSpeed < 255))
  {
    lowerSpeed++;
  }
  else if ((value == 'g' && lowerSpeed > 0) || (value == 'G' && lowerSpeed > 0))
  {
    lowerSpeed--;
  }
  else if (value == '4')
  {
    if (upperSpeed < 246)
      upperSpeed += 10;
    else if (upperSpeed >= 246 && upperSpeed < 255)
      upperSpeed = 255;
  }
  else if (value == 'v' || value == 'V')
  {
    if (upperSpeed > 9)
      upperSpeed -= 10;
    else if (upperSpeed <= 9 && upperSpeed > 0)
      upperSpeed = 0;
  }
  else if (value == '5')
  {
    if (lowerSpeed < 246)
      lowerSpeed += 10;
    else if (lowerSpeed >= 246 && lowerSpeed < 255)
      lowerSpeed = 255;
  }
  else if (value == 'b' || value == 'B')
  {
    if (lowerSpeed > 9)
      lowerSpeed -= 10;
    else if (lowerSpeed <= 9 && lowerSpeed > 0)
      lowerSpeed = 0;
  }

  analogWrite(upperMotorpin, upperSpeed);
  analogWrite(lowerMotorpin, lowerSpeed);
  Serial.print("UPPER SPEED:");
  Serial.println(upperSpeed);
  Serial.print("LOWER SPEED:");
  Serial.println(lowerSpeed);
  /*  upperSensorRead=digitalRead(upperSensorpin);
  lowerSensorRead=digitalRead(lowerSensorpin);
  
  if(upperSensorRead == 1){
    upperIsWhite = 1;
  }else if(upperSensorRead == 0 && upperIsWhite == 1){
      upperRotation++;
    upperIsWhite = 0; 
    }   
    if(lowerSensorRead == 1){
    lowerIsWhite = 1;
  }else if(lowerSensorRead == 0 && lowerIsWhite == 1){
      lowerRotation++;
    lowerIsWhite = 0; 
  }  /*  */
}

int interval_a;
int interval_b;

void tryAA()
{
  //bool_a = false;
  if (millis() - previousMillis_a < interval_a)
    return; // bool_a;
  //bool_a = true;
  previousMillis_a = millis();
}

void tryBB()
{
  //bool_b = false;
  if (millis() - previousMillis_b < interval_b)
    return; // bool_b;
  //bool_b = true;
  previousMillis_b = millis();
}

//HIGHER SPEED NOW!!!!!!!!!!!!!!!!!!!

void fastforward()
{ //1 //Motor move forward .
  analogWrite(_m101, _power2 * _m1_os);
  digitalWrite(_m102, LOW);
  analogWrite(_m201, _power2 * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power2 * _m3_os);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power2 * _m4_os);
}

void backward2()
{ //2 //Motor move backward .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power2 * _m1_os);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power2 * _m2_os);
  analogWrite(_m301, _power2 * _m3_os);
  digitalWrite(_m302, LOW);
  analogWrite(_m401, _power2 * _m4_os);
  digitalWrite(_m402, LOW);
  /* if(millis() - previousMillis > interval){
        stop_all();
          previousMillis = millis();
        } /* */
}

void north_west2()
{ //5 //Motor move to NW .
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  analogWrite(_m201, _power2 * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power2 * _m3_os);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void south_west2()
{ //6 //Motor move to SW .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power2 * _m1_os);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  analogWrite(_m401, _power2 * _m4_os);
  digitalWrite(_m402, LOW);
}

void north_east2()
{ //7 //Motor move to NE .
  analogWrite(_m101, _power2 * _m1_os);
  digitalWrite(_m102, LOW);
  digitalWrite(_m201, HIGH);
  digitalWrite(_m202, HIGH);
  digitalWrite(_m301, HIGH);
  digitalWrite(_m302, HIGH);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power2 * _m4_os);
}

void south_east2()
{ //8 //Motor move to SE .
  digitalWrite(_m101, HIGH);
  digitalWrite(_m102, HIGH);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power2 * _m2_os);
  analogWrite(_m301, _power2 * _m3_os);
  digitalWrite(_m302, LOW);
  digitalWrite(_m401, HIGH);
  digitalWrite(_m402, HIGH);
}

void west2()
{ //9 //Motor move to west .
  digitalWrite(_m101, LOW);
  analogWrite(_m102, _power2 * _m1_os);
  analogWrite(_m201, _power2 * _m2_os);
  digitalWrite(_m202, LOW);
  digitalWrite(_m301, LOW);
  analogWrite(_m302, _power2 * _m3_os);
  analogWrite(_m401, _power2 * _m4_os);
  digitalWrite(_m402, LOW);
}

void east2()
{ //10 //Motor move to east .
  analogWrite(_m101, _power2 * _m1_os);
  digitalWrite(_m102, LOW);
  digitalWrite(_m201, LOW);
  analogWrite(_m202, _power2 * _m2_os);
  analogWrite(_m301, _power2 * _m3_os);
  digitalWrite(_m302, LOW);
  digitalWrite(_m401, LOW);
  analogWrite(_m402, _power2 * _m4_os);
}

int done;

void first_time_SZ_to_LZ()
{ //Function 1 (from Start Zone to Loading Zone)
  forward();
  interval_a = interval_1_forward;
  tryAA();
  stop_all();
}

void LZ_to_TZ1()
{ //Function 2 (from Loading Zone to Throwing Zone 1)
  done = 0;
  if (done == 0)
  {
    backward();
    interval_a = interval_2_backward;
    tryAA();
    done = 1;
  }
  if (done == 1)
  {
    north_east();
    interval_a = interval_2_north_east;
    tryAA();
    stop_all();
  }
}

void TZ1_to_LZ()
{ //Function 3 (from Throwing Zone 1 back to Loading Zone)
  done = 0;
  if (done == 0)
  {
    west();
    interval_a = interval_3_west;
    tryAA();
    done = 1;
  }
  if (done == 1)
  {
    forward();
    interval_a = interval_3_forward;
    tryAA();
    stop_all();
  }
}

void LZ_to_TZ2()
{ //Function 4 (from Loading Zone to Throwing Zone 2)
  done = 0;
  if (done == 0)
  {
    backward();
    interval_a = interval_4_backward;
    tryAA();
    done = 1;
  }
  if (done == 1)
  {
    south_east();
    interval_a = interval_4_south_east;
    tryAA();
    done = 2;
  }
  if (done == 2)
  {
    forward();
    interval_a = interval_4_forward;
    tryAA();
    stop_all();
  }
}

void TZ2_to_LZ()
{ //Function 5 (from Throwing Zone 2 back to Loading Zone)
  done = 0;
  if (done == 0)
  {
    backward();
    interval_a = interval_5_backward;
    tryAA();
    done = 1;
  }
  if (done == 1)
  {
    north_west();
    interval_a = interval_5_north_west;
    tryAA();
    done = 2;
  }
  if (done == 2)
  {
    forward();
    interval_a = interval_5_forward;
    tryAA();
    stop_all();
  }
}

void setup()
{
  Serial.begin(9600); //Serial begin at 9600bps

  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(stepper_top, INPUT);
  pinMode(stepper_bottom, INPUT);

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(a1, HIGH); //1
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH); //2
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //3
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //4
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //5
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //6
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //7
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH); //8
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    if (digitalRead(stepper_top) == HIGH)
    {
      break;
    }
  }

  for (int j = 0; j < 50; j++)
  {
    digitalWrite(a1, HIGH); //8
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds); /*  */

    digitalWrite(a1, LOW); //7
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //6
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //5
    digitalWrite(a2, HIGH);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //4
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, LOW); //3
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH); //2
    digitalWrite(a2, LOW);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);

    digitalWrite(a1, HIGH); //1
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, LOW);
    delayMicroseconds(seconds);
    if (digitalRead(stepper_bottom) == HIGH)
    {
      break;
    }
  }

  previousMillis_a = millis();
  previousMillis_b = millis();

  /*pinMode(upperSensorpin, INPUT);  //For 2 shooting ball wheels and 7 segments display setups
  pinMode(lowerSensorpin, INPUT);*/
  pinMode(upperMotorpin, OUTPUT);
  pinMode(lowerMotorpin, OUTPUT);

  pinMode(_m101, OUTPUT); //For teensymotor's setup
  pinMode(_m102, OUTPUT);
  pinMode(_m201, OUTPUT);
  pinMode(_m202, OUTPUT);
  pinMode(_m301, OUTPUT);
  pinMode(_m302, OUTPUT);
  pinMode(_m401, OUTPUT);
  pinMode(_m402, OUTPUT);

  myservo.attach(servo_pin);  //For servo's setup
  myservo.write(servo_lower); //Servo begin at 20 degrees positions
}

int ok_ok = 0;
void loop()
{
  if (Serial.available() > 0)
  {

    value = Serial.read(); //Enter a value for controlling the robot
    Serial.println(value);

    shotting_ball_2_wheels();

    switch (value)
    {

    //Normal speed
    case 'w': //Motor move forward
      forward();
      break;
    case 'x': //Motor move backward
      backward();
      break;
    case '1': //Motor turn left
      left();
      break;
    case '3': //Motor turn right
      right();
      break;
    case 'q': //Motor move to NW
      north_west();
      break;
    case 'z': //Motor move to SW
      south_west();
      break;
    case 'e': //Motor move to NE
      north_east();
      break;
    case 'c': //Motor move to SE
      south_east();
      break;
    case 'a': //Motor move to west
      west();
      break;
    case 'd': //Motor move to east
      east();
      break;

    //Faster speed
    case 'W': //Motor move forward
      fastforward();
      break;
    case 'X': //Motor move backward
      backward2();
      break;
    case 'Q': //Motor move to NW
      north_west2();
      break;
    case 'Z': //Motor move to SW
      south_west2();
      break;
    case 'E': //Motor move to NE
      north_east2();
      break;
    case 'C': //Motor move to SE
      south_east2();
      break;
    case 'A': //Motor move to west
      west2();
      break;
    case 'D': //Motor move to east
      east2();
      break;
    case 's': //Motor stop moving
      stop_all();
      break;
    case 'S': //Motor stop moving
      stop_all();
      break;
    case ' ': //Motor stop moving
      stop_all();
      break;

    //Servo
    case '=': //Servo move up and down
      servo_move();
      break;

    //Stepper
    case 'h': //Stepper move downward
      stepper_downward();
      break;
    case 'n': //Stepper move downward
      stepper_downward_high();
      break;
    case 'y': //Stepper move upward
      stepper_upward();
      break;
    case '6': //Stepper move upward
      stepper_upward_high();
      break;
    case 'H': //Stepper move downward
      stepper_downward();
      break;
    case 'N': //Stepper move downward
      stepper_downward_high();
      break;
    case 'Y': //Stepper move upward
      stepper_upward();
      break;
    case 'l':
      north_east_soft();
      break;
    case 'L':
      north_east_soft();
      break;
    case '7': //Function 1 Start Zone ~ Loading Zone
      first_time_SZ_to_LZ();
      break;
    case '8': //Function 2 Loading Zone ~ Throwing Zone 1
      LZ_to_TZ1();
      break;
    case '9': //Function 3 Throwing Zone 1 ~ Loading Zone
      TZ1_to_LZ();
      break;
    case '0': //Function 4 Loading Zone ~ Throwing Zone 2
      LZ_to_TZ2();
      break;
    case '-': //Function 5 Throwing Zone 2 ~ Loading Zone
      TZ2_to_LZ();
      break;

    default: //Return to motor stop motion and servo stay at 20 degree
      stop_all();
      myservo.write(20);
      break;
    }
  }
  delay(500);
  stop_all();
}
