//IN HARDWARE CIRCUIT --->  10(DIRECTION),11(STEP),12(ENABLE) IS CONNECTED TO TILT MOTOR DRIVER.//
//IN HARDWARE CIRCUIT --->   2(DIRECTION), 3(STEP), 6(ENABLE) IS CONNECTED TO PAN MOTOR DRIVER.//
#include <AccelStepper.h>

//Setup - not configurable
#define motorInterfaceType    1
#define EastSensor            4  // East Hall sensor for Interrupt
#define WestSensor            5  // West Hall sensor for Interrupt

#define TiltMotorStepPin      11
#define TiltMotorDirectionPin 10
#define TiltMotorEnable       12

#define PanMotorStepPin       3
#define PanMotorDirectionPin  2
#define PanMotorEnable        6


//Cofigurable Max Speed, Current Speed and Number of steps motor rotates on getting one command
#define TimeToWait            500        //32767 this is for 9 hour
#define MaxSpeed              200
#define CurrentSpeed          20
#define PanMotorReturnSpeed   100
#define NumbersOfStepsAtOnce  15


#define PanMotorNumberOfStepsForReturn 200

//Partially configurable, i.e. toggle 1 & -1 if complete system is rotating in opposite directions
#define Clockwise                  1
#define AntiClockwise             -1
#define PanMotorReturnDirection   -1

//Configurable Sensor values
#define SensorThreshold 3
#define FunctionalLightConditionValue 500

//Stepper motor instances
AccelStepper TiltStepperMotor = 
  AccelStepper(motorInterfaceType, TiltMotorStepPin, TiltMotorDirectionPin); // replaced by PanStepperMotor

AccelStepper PanStepperMotor = 
  AccelStepper(motorInterfaceType, PanMotorStepPin, PanMotorDirectionPin);  //  replaced by TiltStepperMotor

int EastSideInterrupt = 0;
int WestSideInterrupt = 0;
int timeElapsedSinceNotFunctionalLightConditions = 0;

void setup() 
{
  PanStepperMotor.setMaxSpeed(MaxSpeed); 
  TiltStepperMotor.setMaxSpeed(MaxSpeed);  
  
  Serial.begin(9600);
  analogReference(INTERNAL);
  
  pinMode(EastSensor, INPUT);
  pinMode(WestSensor, INPUT);  
}

void loop() 
{ 
  int topLeft     = analogRead(A0);
  int topRight    = analogRead(A1);
  int bottomLeft  = analogRead(A2);
  int bottomRight = analogRead(A3);
  
  int avgLeft     = (topLeft    + bottomLeft)   / 2;
  int avgRight    = (topRight   + bottomRight)  / 2;
  int avgTop      = (topLeft    + topRight)     / 2;
  int avgBottom   = (bottomLeft + bottomRight)  / 2;
  
  SerialPrintSensorValues(topLeft, topRight, bottomLeft, bottomRight);
  
  if(functionalLightConditions(topLeft, topRight, bottomLeft, bottomRight))
  {
    timeElapsedSinceNotFunctionalLightConditions = 0;
    
    //following "if" block will be executed when the system 
    //has executed a return procedure earlier or
    //is a fresh start
    if(IsEastSideInterruptEngaged() == 1)
    {
      EastSideInterrupt = 0;
      WestSideInterrupt = 0;
      timeElapsedSinceNotFunctionalLightConditions = 0;
    }
    
    if(IsWestSideInterruptEngaged() == 0)
    {
      trackSun(avgLeft, avgRight, avgTop, avgBottom);
    }
    else     
    {
      turnOffAllMotors();
    }
  }
  else //if(!functionalLightConditions(topLeft, topRight, bottomLeft, bottomRight))
  {
    if(IsEnoughTimeElapsed() && IsEastSideInterruptEngaged() == 0)
    {
      //start rotating pan motor till East Side Interrupt is engaged
      executeReturnProcedure();
    }
    turnOffAllMotors();
  }
}

bool functionalLightConditions(int topLeft, int topRight, int bottomLeft, int bottomRight)
{
  //lesser the values higher is the light intensity
  if(topLeft < FunctionalLightConditionValue && topRight < FunctionalLightConditionValue 
    && bottomLeft < FunctionalLightConditionValue && bottomRight < FunctionalLightConditionValue)
    return true;
  return false;
}

bool IsEnoughTimeElapsed()
{
  if(timeElapsedSinceNotFunctionalLightConditions == TimeToWait)
  {
    return true;
  }
  timeElapsedSinceNotFunctionalLightConditions++;
  return false;
}

int IsWestSideInterruptEngaged()
{
  //As the Pan Motor starts returning to east direction, we will no longer get the interrupt signal 
  //but WestSideInterrupt will remember if we got the sun set conditions earlier
  //and this flag (WestSideInterrupt) should be returned immidiately
  if(WestSideInterrupt == 1)
    return WestSideInterrupt;
    
   WestSideInterrupt = digitalRead(WestSensor);
  
  return WestSideInterrupt;
}

int IsEastSideInterruptEngaged()
{
  if(EastSideInterrupt == 1)
    return EastSideInterrupt;
  
  EastSideInterrupt = digitalRead(EastSensor);
  
  return EastSideInterrupt;
}

void trackSun(int avgLeft, int avgRight, int avgTop, int avgBottom)
{
  int PanDirection  = 0;
  int TiltDirection = 0;
  
  if(avgLeft - avgRight > SensorThreshold)
  {
    PanDirection = AntiClockwise;
    
    //Start Motors
    digitalWrite(PanMotorEnable, LOW);
  }
  else if(avgRight - avgLeft > SensorThreshold)
  {
    PanDirection = Clockwise;
    //Start Motors
    digitalWrite(PanMotorEnable, LOW);
  }
  else
  {
    PanDirection = 0;
    //Shutdown Motors
    digitalWrite(PanMotorEnable, HIGH);
  }
  
  if(avgTop - avgBottom > SensorThreshold)
  {
    TiltDirection = Clockwise;
    //Start Motors
    digitalWrite(TiltMotorEnable, LOW);
  }
  else if(avgBottom - avgTop > SensorThreshold)
  {
    TiltDirection = AntiClockwise;
    //Start Motors
    digitalWrite(TiltMotorEnable, LOW);
  }
  else
  {
    TiltDirection = 0;
    //Shutdown Motors
    digitalWrite(TiltMotorEnable, HIGH);
  }

  //At this point respective motors will be switched on or off, (in case of off, motors won't respond to any further commands)
  //Pan and Tilt direction will also be set to clock, anti-clock or 0
  
  TiltStepperMotor.setCurrentPosition(0);
  PanStepperMotor.setCurrentPosition(0);
  while(TiltStepperMotor.currentPosition() != (NumbersOfStepsAtOnce * TiltDirection) || PanStepperMotor.currentPosition() != (NumbersOfStepsAtOnce * PanDirection))
  {
    if(TiltStepperMotor.currentPosition() != (NumbersOfStepsAtOnce * TiltDirection))
    {
      TiltStepperMotor.setSpeed(CurrentSpeed * TiltDirection);
      TiltStepperMotor.runSpeed();
    }
    
    if(PanStepperMotor.currentPosition() != (NumbersOfStepsAtOnce * PanDirection))
    {
      PanStepperMotor.setSpeed(CurrentSpeed * PanDirection);
      PanStepperMotor.runSpeed();
    } 
  }
}

void executeReturnProcedure()
{
  digitalWrite(PanMotorEnable, LOW);
  
  PanStepperMotor.setCurrentPosition(0);
  while(PanStepperMotor.currentPosition() != (PanMotorNumberOfStepsForReturn * PanMotorReturnDirection))
  {
    PanStepperMotor.setSpeed(PanMotorReturnSpeed * PanMotorReturnDirection);
    PanStepperMotor.runSpeed();
  }
  
  digitalWrite(PanMotorEnable, HIGH);
}

void turnOffAllMotors()
{
  digitalWrite(TiltMotorEnable, HIGH);
  digitalWrite(PanMotorEnable, HIGH);
}

void SerialPrintSensorValues(int topLeft, int topRight, int bottomLeft, int bottomRight)
{
  Serial.println();
  Serial.print("Top Left = ");
  Serial.print(topLeft);
  Serial.print("\t Top Right = ");
  Serial.print(topRight);
  Serial.print("\t Bottom Left = ");
  Serial.print(bottomLeft);
  Serial.print("\t Bottom Right = ");
  Serial.print(bottomRight);

  Serial.print("EastSensor = ");
  Serial.println(EastSideInterrupt); 
  
  Serial.print("WestSensor = ");
  Serial.println(WestSideInterrupt);
  
  Serial.print("Time = ");
  Serial.println(timeElapsedSinceNotFunctionalLightConditions);
  
  /*
  Serial.println();
  Serial.print("Pan Stepper Motor position = ");
  Serial.print(PanStepperMotor.currentPosition());
  Serial.print("\tTilt Stepper Motor position = ");
  Serial.print(TiltStepperMotor.currentPosition());
  */
}
