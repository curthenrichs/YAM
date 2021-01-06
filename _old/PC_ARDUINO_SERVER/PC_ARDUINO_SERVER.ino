//server for roboserver
//command structure is <command>:<Parm1>,<Parm2>,...;

//right motor
const int IN1 = 8;
const int EA = 9;
const int IN2 = 11;

//left Motor
const int EB = 10;
const int IN3 = 12;
const int IN4 = 13;

//client inpput variables

String input = "";
String command = "";

//motor values 
byte leftSpeed = 0;
byte rightSpeed = 0;
byte leftOut = 0;
byte rightOut = 0;
char leftDir = 'f';
char rightDir = 'r';

//PID closed loop varibles
double P = 0;
double I = 0;
double D = 0;

int leftEnc;
int rightEnc;

void setup()
{
//Motor setup and enable
  //the right motor
  pinMode(EA,OUTPUT);
  digitalWrite(EA,HIGH);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  //the left motor
  pinMode(EB,OUTPUT);
  digitalWrite(EB,HIGH);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  //set up communication bus
  Serial.begin(9600);
  
  //prime copmmands line
  char q = '0';
  
  command = "M:f," + q;
  command += ",f,";
  command += q;
  command += ";";
}

void loop()
{
  //get value from over serial for what we are to do
  serialInput();
  
  //this part will be for implementing all of the different cases
  //that we can use like set open/ closed loop
  //position/ speed mode iif in closed loop
  //and also set motor speeds and look up encoder speeds/position
  //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
  
  
  //motorCommands
  if(command.charAt(0) == 'M')
  {
    leftDir= command.charAt(2);
    leftSpeed = (byte)command.charAt(4);
    
    rightDir = command.charAt(6);
    rightSpeed = (byte)command.charAt(8);
    
  }
  else if(command.charAt(0) == 'P')
  {
    //toggle on off closed loop 
  }
  else //stop the motors //default command will be S;
  {
    leftDir = 'f';
    leftSpeed = 0;
    
    rightDir = 'f';
    rightSpeed = 0;
  }
  
  
  //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
  
  
  //setup motor values for output
  rightMotorSet();
  leftMotorSet();
  
  //output motors
  rightMotorOut();
  leftMotorOut();
  
  //reset values to zero
  rightOut = 0;
  leftOut = 0;
  
  
  //this is  where we send out the data for current speed and time stamp
  
  //delay for buffer to refill
  delay(500);
}
//========================<Subs>================================

//--------------------------------------------------------------
void serialInput()
{
  while(Serial.available() > 0)
  {
    byte temp = Serial.read();
    
    if(!(temp == ';'))
    {
      input += (char)temp;
    }
    else
    {
      command = input;
      input = "";
    }
  }
}

//-------------------------------------------------------------
void rightMotorSet()
{
  rightOut =  rightSpeed - 1; 
   
  rightOut =  rightOut * 56.0/254.0;

  if(rightOut <= 0)
  {
     rightOut = 0;
  }
  else
  {
    rightOut += 200;
  }  
}

//----------------------------------------------------------------
void leftMotorSet()
{
  leftOut = leftSpeed - 1;
  
  leftOut = leftOut * 56.0/255.0;
  
  if(leftOut <= 0)
  {
    leftOut = 0;
  }
  else
  {
    leftOut += 200;
  }
}

//---------------------------------------------------------------
void rightMotorOut()
{
    //right motor
  if(rightDir == 'f')
  {
    analogWrite(IN1,0);
    analogWrite(IN2,rightOut);
  }
  else
  {
    analogWrite(IN1,rightOut);
    analogWrite(IN2,0);
  }
}

//---------------------------------------------------------------
void leftMotorOut()
{
    //left motor
  if(leftDir == 'f')
  {
    analogWrite(IN3,0);
    analogWrite(IN4,leftOut);
  }
  else
  {
    analogWrite(IN3,leftOut);
    analogWrite(IN4,0);
  }
}
