#include <Arduino.h>
#include "xmotionV3.h"
#include "alin.h"

#define SEARCH_SPEED 30
#define COMBAT_SPEED 90

#define LINE_READ_THRESHOLD 500
//below -> WHITE
//above -> BLACK

enum ProgramType
{
  Alin,
  Camila
};

ProgramType program;

#pragma region PIN_DEFINITION

#define LEFT_LINESENSOR_PIN A5
#define RIGHT_LINESENSOR_PIN A4
#define TELECOM_PIN A0

#define RM_DIR 13
#define RM_PWM 11
#define LM_DIR 12
#define LM_PWM 10

#pragma endregion

XMotionClass motion;

#pragma region SENSOR_READINGS
//sensor readings
int leftIR = 0;
int rightIR = 0;
int frontLeftIR = 0;
int frontMidIR = 0;
int frontRightIR = 0;

int leftLineRead = 1000;
int rightLineRead = 1000;

int telecomRead = 0;
bool robot_ready_state = false;
bool initial_start_pin_state = false;
#pragma endregion

#pragma region FUNCTION_DECLARATIONS

//IR Sensor reading
int LeftIR();
int RightIR();
int FrontRightIR();
int FrontLeftIR();
int FrontMidIR();

void ReadIRSensorInfo();


//LINE Sensor reading
void ReadLineSensorInfo();



//enemy detection functionns
bool SeesAnything();

bool EnemyIsRight();
bool EnemyIsLeft();
bool EnemyIsFront();


//directional movement functions
void TurnAroundRight(int RightSpeed , int time);

void TurnForwardRight(int RightSpeed , int time);
void TurnBackRight(int Speed,int time);

//RUSH B DO NOT STOP
void RushB(int RushBSpeed,int time);

void Execute();

#pragma endregion

void setup() {

  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(TELECOM_PIN,INPUT);
  
  if(digitalRead(5))
  {
    //program alin
    alin_init();
    program = ProgramType::Alin;
  }else if(digitalRead(6))
  {
    //program andrei
      pinMode(0, INPUT); // - ir stanga sasiu
      pinMode(1, INPUT); // - ir stanga frontal
      pinMode(2, INPUT); // - ir fata frontal
      pinMode(3, INPUT); // - ir dreapta sasiu
      pinMode(4, INPUT); // - ir frontal dreapta

      pinMode(LEFT_LINESENSOR_PIN,INPUT);
      pinMode(RIGHT_LINESENSOR_PIN,INPUT);

      program = ProgramType::Camila;
  }else if(digitalRead(7))
  {
    //pizdec??
    while(true){}
  }else
  {
    while(true){}
  }




}

void loop() {
  //Robochallange();

  if(program == ProgramType::Alin)
  {
    str3();
  }else if(program == ProgramType::Camila)
  {
    Execute();
  }
}

void Robochallange()
{
  // initial check blyat
  initial_start_pin_state = digitalRead(TELECOM_PIN);
  if (initial_start_pin_state == 1)
  {
    // error BLYAT,PIZDEC
    while (true)
    {
      // blyat,RESET,CYKA
    }
  }
  else if (initial_start_pin_state == 0)
  {
    robot_ready_state = true;
  }

  int current_start_pin_state = 0;
  while (current_start_pin_state == 0)
  {
    // keep the robot locked in a loop until the start command  is received

    current_start_pin_state = digitalRead(TELECOM_PIN);
  }

  // the program will continue only if the start command is received

  while (current_start_pin_state == 1)
  {
    // do program loop
    if(program == ProgramType::Alin)
    {
      str3();
    }else if(program == ProgramType::Camila)
    {
      Execute();
    }
    

    current_start_pin_state = digitalRead(TELECOM_PIN);
  }

  // the program will continue only if the stop command is received
  if (current_start_pin_state == 0)
  {

    // the stop command was received
    // this is where the robot should stop all operations

    while (true)
    {

      // for safety reasons the robot is locked in this loop
      // you can flash a separate LED to let the user know
      // the only way to exit this loop is to do a power cycle
    }
  }
}

void ReadIRSensorInfo()
{
  leftIR       = LeftIR();
  rightIR      = RightIR();
  frontLeftIR  = FrontLeftIR();
  frontMidIR   = FrontMidIR();
  frontRightIR = FrontRightIR();

  Serial.println(SeesAnything());
}

void ReadLineSensorInfo()
{
  leftLineRead = analogRead(LEFT_LINESENSOR_PIN);
  rightLineRead = analogRead(RIGHT_LINESENSOR_PIN);
}

#pragma region SensorReadingFunctions

int LeftIR()
{
  return digitalRead(0);
}

int RightIR()
{
  return digitalRead(3);
}

int FrontRightIR()
{
  return digitalRead(4);
}

int FrontLeftIR()
{
  return digitalRead(1);
}

int FrontMidIR()
{
  return digitalRead(2);
}

#pragma endregion

bool SeesWhite()
{
  return (leftLineRead < LINE_READ_THRESHOLD || rightLineRead < LINE_READ_THRESHOLD);
}

bool SeesAnything()
{
    return(leftIR || rightIR || frontLeftIR || frontMidIR || frontRightIR);
}

bool EnemyIsRight()
{
    return (rightIR || frontRightIR);
}

bool EnemyIsLeft()
{
  return(leftIR || frontLeftIR);
}

bool EnemyIsFront()
{
  //return (frontMidIR);
  return ((frontMidIR && frontLeftIR) || (frontMidIR && frontRightIR));
}

void TurnAroundRight(int RightSpeed , int time)
{
  digitalWrite(LM_DIR,LOW);

  RightSpeed = map(RightSpeed,0,100,0,255);
  analogWrite(LM_PWM , RightSpeed);
  analogWrite(RM_PWM,0);
  delay(time);
}

void TurnBackRight(int Speed,int time)
{
  digitalWrite(LM_DIR,LOW);
  digitalWrite(RM_DIR,HIGH);

  Speed = map(Speed,0,100,0,255);

  analogWrite(LM_PWM , Speed);
  analogWrite(RM_PWM , Speed);

  delay(time);
}

void TurnForwardRight(int RightSpeed, int time)
{
  digitalWrite(LM_DIR,HIGH);
  digitalWrite(RM_DIR,LOW);

  RightSpeed = map(RightSpeed,0,100,0,255);
  analogWrite(LM_PWM , RightSpeed);
  analogWrite(RM_PWM,  RightSpeed);
  delay(time);
}

void RushB(int RushBSpeed, int time)
{
  digitalWrite(RM_DIR,HIGH);
  digitalWrite(LM_DIR,HIGH);

  RushBSpeed = map(RushBSpeed,0,100,0,255);

  analogWrite(LM_PWM , RushBSpeed);
  analogWrite(RM_PWM , RushBSpeed);
}

void Execute()
{
//store IR sensor data

  ReadIRSensorInfo();
  ReadLineSensorInfo();

  if(!SeesAnything())
  {
    //while no enemy detected,rotate around like a jerk
    TurnAroundRight(15 , 50);
    return;
  }

  else
  {
    if(EnemyIsFront())
    {
      //Rush B

      RushB(90,50);

      return;
    }

    //rotate towards enemy
    else if(EnemyIsRight())
    {
      //rotate right
      motion.StopMotors(1);
      TurnForwardRight(15,1);
      
      return;

    }else if(EnemyIsLeft())
    {
      //rotate left
      motion.StopMotors(1);
      TurnBackRight(15,1);
      return;
    }
  }
}
