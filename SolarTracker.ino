//IN HARDWARE CIRCUIT --->  10(DIRECTION),11(STEP),12(ENABLE) IS CONNECTED TO TILT MOTOR DRIVER.//
//IN HARDWARE CIRCUIT --->   2(DIRECTION), 3(STEP), 6(ENABLE) IS CONNECTED TO PAN MOTOR DRIVER.//


#include <AccelStepper.h>
int i=0;

#define MaxInt 50 //32767 this is for 9 hour

//Setup - not configurable
#define motorInterfaceType    1

#define TiltMotorStepPin      3 //3 replaced by PanMotorStepPin
#define TiltMotorDirectionPin 2//2  replaced by PanMotorDirectionPin
#define TiltMotorEnable       6 //6 replaced by PanMotorEnable

#define PanMotorStepPin       11 // 11replaced by TiltMotorStepPin (5 pin replaced by 11 pin)
#define PanMotorDirectionPin  10 // 10replaced by TiltMotorDirectionPin (4 pin replaced by 10 pin)
#define PanMotorEnable        12 //12 replaced by TiltMotorEnable(7 pin replaced by 12 pin)

#define EastSensor             4 // East Hall sensor for Interrupt

#define WestSensor             5 // West Hall sensor for Interrupt


//Cofigurable Max Speed, Current Speed and Number of steps motor rotates on getting one command
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

bool IsEastSideInterruptEngaged = false;
bool IsWestSideInterruptEngaged = false;
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
  IsEastSideInterruptEngaged = digitalRead(EastSensor);
  IsWestSideInterruptEngaged = digitalRead(WestSensor);
  int topLeft     = analogRead(A0);
  int topRight    = analogRead(A1);
  int bottomLeft  = analogRead(A2);
  int bottomRight = analogRead(A3);
  
  int avgLeft     = (topLeft    + bottomLeft)   / 2;
  int avgRight    = (topRight   + bottomRight)  / 2;
  int avgTop      = (topLeft    + topRight)     / 2;
  int avgBottom   = (bottomLeft + bottomRight)  / 2;

  SerialPrintSensorValues(avgLeft, avgRight, avgTop, avgBottom);
   if(IsEastSideInterruptEngaged)
{
  i=1;  //timeElapsedSinceNotFunctionalLightConditions = 0;  
  }
  else
  {
    i=i;  //timeElapsedSinceNotFunctionalLightConditions = timeElapsedSinceNotFunctionalLightConditions;
   }

    if(IsWestSideInterruptEngaged)
{
    //timeElapsedSinceNotFunctionalLightConditions = 0;  
  }
  else
  {
      //timeElapsedSinceNotFunctionalLightConditions = timeElapsedSinceNotFunctionalLightConditions;
   }
  
   Serial.print("EastSensor = ");
   Serial.println(IsEastSideInterruptEngaged); 
  
    Serial.print("WestSensor = ");
    Serial.println(IsWestSideInterruptEngaged);

    Serial.print("Time = ");
   Serial.println(timeElapsedSinceNotFunctionalLightConditions);

   Serial.print(" Anshul = ");
   Serial.println(i);

  
  if(functionalLightConditions(topLeft, topRight, bottomLeft, bottomRight))
  {
    timeElapsedSinceNotFunctionalLightConditions = 0;
    i=0;
    //following "if" block will be executed when the system 
    //has executed a return procedure earlier or
    //is a fresh start
    if(IsEastSideInterruptEngaged == 1)
    {
      IsEastSideInterruptEngaged = 0;
      IsWestSideInterruptEngaged = 0;
      timeElapsedSinceNotFunctionalLightConditions = 0; 
      i=0;
    }

    else
    {
      
    }
    
    if(IsWestSideInterruptEngaged == 0)
    {
      trackSun(avgLeft, avgRight, avgTop, avgBottom);
    }
   else           //if(EastSideInterruptIsEngaged())
    {
      turnOffAllMotors();   //Need to do anything?
    }
  }
  else //if(!functionalLightConditions(topLeft, topRight, bottomLeft, bottomRight))
  {
    if(IsEnoughTimeElapsed()&& (i==0) && (IsEastSideInterruptEngaged == 0))
    {
      //start rotating pan motor till EastSideInterruptIsEngaged
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
  if(timeElapsedSinceNotFunctionalLightConditions == MaxInt)
  {
    return true;
  }
  timeElapsedSinceNotFunctionalLightConditions++;
  return false;
}

bool WestSideInterruptIsEngaged()
{
  //As the Pan Motor starts returning to east direction, we will no longer get the interrupt signal 
  //but IsWestSideInterruptEngaged will remember if we got the sun set conditions earlier
  //and this flag (IsWestSideInterruptEngaged) should be returned immidiately
  if(WestSideInterruptIsEngaged)
    return IsWestSideInterruptEngaged;
    
  //ToDo: Check if west side interrupt pin is set/engaged
  //if set
  //then 
  //  IsWestSideInterruptEngaged = true;
  //else
  //  IsWestSideInterruptEngaged = false;
  
  return IsWestSideInterruptEngaged;
}

bool EastSideInterruptIsEngaged()
{
  if(IsEastSideInterruptEngaged)
 
    return IsEastSideInterruptEngaged;
    
  //ToDo: Check if east side interrupt pin is set/engaged
  //if set
  //then 
  //  IsEastSideInterruptEngaged = true;
  //else
  //  IsEastSideInterruptEngaged = false;
  
  return IsEastSideInterruptEngaged;
}

void trackSun(int avgLeft, int avgRight, int avgTop, int avgBottom)
{
  int PanDirection  = 0;
  int TiltDirection = 0;
  
  if(avgLeft - avgRight > SensorThreshold)
  {
    TiltDirection = AntiClockwise;
    
    //Start Motors
    digitalWrite(TiltMotorEnable, LOW);
  }
  else if(avgRight - avgLeft > SensorThreshold)
  {
    TiltDirection = Clockwise;
    
    //Start Motors
    digitalWrite(TiltMotorEnable, LOW);
  }
  else
  {
    TiltDirection = 0;
    //Shutdown Motors
    digitalWrite(TiltMotorEnable, HIGH);
  }
  
  if(avgTop - avgBottom > SensorThreshold)
  {
    PanDirection = Clockwise;
    //Start Motors
    digitalWrite(PanMotorEnable, LOW);
  }
  else if(avgBottom - avgTop > SensorThreshold)
  {
    PanDirection = AntiClockwise;
    //Start Motors
    digitalWrite(PanMotorEnable, LOW);
  }
  else
  {
    PanDirection = 0;
    //Shutdown Motors
    digitalWrite(PanMotorEnable, HIGH);
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

void SerialPrintSensorValues(int avgLeft, int avgRight, int avgTop, int avgBottom)
{
  Serial.println();
  Serial.print("Avg Left = ");
  Serial.print(avgLeft);
  Serial.print("\t Avg Right = ");
  Serial.print(avgRight);
  Serial.print("\t Avg Top = ");
  Serial.print(avgTop);
  Serial.print("\t Avg Bottom = ");
  Serial.print(avgBottom);
  
  /*
  Serial.println();
  Serial.print("Pan Stepper Motor position = ");
  Serial.print(PanStepperMotor.currentPosition());
  Serial.print("\tTilt Stepper Motor position = ");
  Serial.print(TiltStepperMotor.currentPosition());
  */
}
