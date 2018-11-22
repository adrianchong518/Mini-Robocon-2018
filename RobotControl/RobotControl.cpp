#include <RobotControl.h>

void robotSetup()
{
  // Aiming System - Motor Pins
  pinMode(UPPER_MOTOR_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_PIN, OUTPUT);

  // Aiming System - Stepper
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(B_2, OUTPUT);
  pinMode(STEPPER_TOP, INPUT);
  pinMode(STEPPER_BOTTOM, INPUT);

  // Aiming System - Ball Loading
  loadingServo.attach(SERVO_PIN);
  loadingServo.write(SERVO_LOWER);

  // Movement System - Motor Pins
  pinMode(M101, OUTPUT);
  pinMode(M102, OUTPUT);
  pinMode(M201, OUTPUT);
  pinMode(M202, OUTPUT);
  pinMode(M301, OUTPUT);
  pinMode(M302, OUTPUT);
  pinMode(M401, OUTPUT);
  pinMode(M402, OUTPUT);
}

// Aiming System - Stepper Control
int setState(int inc)
{
  // Check if already topped or bottomed out
  if ((digitalRead(STEPPER_TOP) || digitalRead(STEPPER_BOTTOM)) == HIGH)
    return 1;

  // Increament the state counter
  stateCounter += inc;
  stateCounter %= STATE_NUM;
  int state = STATES[stateCounter];

  // Write the state to stepper
  digitalWrite(A_1, (state & 0b0001) >> 0);
  digitalWrite(A_2, (state & 0b0010) >> 1);
  digitalWrite(B_1, (state & 0b0100) >> 2);
  digitalWrite(B_2, (state & 0b1000) >> 3);
  delayMicroseconds(DELAY_US);

  reutrn 0;
}

int turnServo(int inc, int numStates)
{
  for (int i = 0; i < numStates; i++)
    if (setState(inc))
      return 1;

  return 0;
}

// Aiming System - Ball Loading
void loadBall()
{
  loadingServo.write(SERVO_UPPER);
  delay(DELAY_BEFORE_DEC);
  loadingServo.write(SERVO_LOWER);
}

// Movement System - Directions
void moveN(int power)
{
  analogWrite(M101, power * M1_OS);
  digitalWrite(M102, LOW);
  analogWrite(M201, power * M2_OS);
  digitalWrite(M202, LOW);
  digitalWrite(M301, LOW);
  analogWrite(M302, power * M3_OS);
  digitalWrite(M401, LOW);
  analogWrite(M402, power * M4_OS);
}

void moveS(int power)
{
  digitalWrite(M101, LOW);
  analogWrite(M102, power * M1_OS);
  digitalWrite(M201, LOW);
  analogWrite(M202, power * M2_OS);
  analogWrite(M301, power * M3_OS);
  digitalWrite(M302, LOW);
  analogWrite(M401, power * M4_OS);
  digitalWrite(M402, LOW);
}

void moveE(int power)
{
  analogWrite(M101, power * M1_OS);
  digitalWrite(M102, LOW);
  digitalWrite(M201, LOW);
  analogWrite(M202, power * M2_OS);
  analogWrite(M301, power * M3_OS);
  digitalWrite(M302, LOW);
  digitalWrite(M401, LOW);
  analogWrite(M402, power * M4_OS);
}

void moveW(int power)
{
  digitalWrite(M101, LOW);
  analogWrite(M102, power * M1_OS);
  analogWrite(M201, power * M2_OS);
  digitalWrite(M202, LOW);
  digitalWrite(M301, LOW);
  analogWrite(M302, power * M3_OS);
  analogWrite(M401, power * M4_OS);
  digitalWrite(M402, LOW);
}

void moveNE(int power)
{
  analogWrite(M101, power * M1_OS);
  digitalWrite(M102, LOW);
  digitalWrite(M201, HIGH);
  digitalWrite(M202, HIGH);
  digitalWrite(M301, HIGH);
  digitalWrite(M302, HIGH);
  digitalWrite(M401, LOW);
  analogWrite(M402, power * M4_OS);
}

void moveNW(int power)
{
  digitalWrite(M101, HIGH);
  digitalWrite(M102, HIGH);
  analogWrite(M201, power * M2_OS);
  digitalWrite(M202, LOW);
  digitalWrite(M301, LOW);
  analogWrite(M302, power * M3_OS);
  digitalWrite(M401, HIGH);
  digitalWrite(M402, HIGH);
}

void moveSE(int power)
{
  digitalWrite(M101, HIGH);
  digitalWrite(M102, HIGH);
  digitalWrite(M201, LOW);
  analogWrite(M202, power * M2_OS);
  analogWrite(M301, power * M3_OS);
  digitalWrite(M302, LOW);
  digitalWrite(M401, HIGH);
  digitalWrite(M402, HIGH);
}

void moveSW(int power)
{
  digitalWrite(M101, LOW);
  analogWrite(M102, power * M1_OS);
  digitalWrite(M201, HIGH);
  digitalWrite(M202, HIGH);
  digitalWrite(M301, HIGH);
  digitalWrite(M302, HIGH);
  analogWrite(M401, power * M4_OS);
  digitalWrite(M402, LOW);
}

// Movement System - Rotation
void rotCW(int power)
{
  digitalWrite(M101, LOW);
  digitalWrite(M102, LOW);
  digitalWrite(M201, LOW);
  digitalWrite(M202, LOW);
  digitalWrite(M301, LOW);
  analogWrite(M302, power * M3_OS);
  digitalWrite(M401, LOW);
  analogWrite(M402, power * M4_OS);
}

void rotCCW(int power)
{
  analogWrite(M101, power * M1_OS);
  digitalWrite(M102, LOW);
  analogWrite(M201, power * M2_OS);
  digitalWrite(M202, LOW);
  digitalWrite(M301, LOW);
  digitalWrite(M302, LOW);
  digitalWrite(M401, LOW);
  digitalWrite(M402, LOW);
}

void selfRotCW(int power)
{
  digitalWrite(M101, LOW);
  analogWrite(M102, power * M1_OS);
  digitalWrite(M201, LOW);
  analogWrite(M202, power * M2_OS);
  digitalWrite(M301, LOW);
  analogWrite(M302, power * M3_OS);
  digitalWrite(M401, LOW);
  analogWrite(M402, power * M4_OS);
}

void selfRotCCW(int power)
{
  analogWrite(M101, power * M1_OS);
  digitalWrite(M102, LOW);
  analogWrite(M201, power * M2_OS);
  digitalWrite(M202, LOW);
  analogWrite(M301, power * M3_OS);
  digitalWrite(M302, LOW);
  analogWrite(M401, power * M4_OS);
  digitalWrite(M402, LOW);
}

void moveStop()
{
  digitalWrite(M101, HIGH);
  digitalWrite(M102, HIGH);
  digitalWrite(M201, HIGH);
  digitalWrite(M202, HIGH);
  digitalWrite(M301, HIGH);
  digitalWrite(M302, HIGH);
  digitalWrite(M401, HIGH);
  digitalWrite(M402, HIGH);
}