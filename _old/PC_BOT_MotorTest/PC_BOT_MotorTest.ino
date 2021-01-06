//D8 IN1
//D9 EA
//D10 EB
//D11 IN2
//D12 In3
//D13IN4 

//right motor
const int IN1 = 8;
const int EA = 9;
const int IN2 = 11;

//left Motor
const int EB = 10;
const int IN3 = 12;
const int IN4 = 13;

void setup()
{
  pinMode(EA,OUTPUT);
  digitalWrite(EA,HIGH);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  pinMode(EB,OUTPUT);
  digitalWrite(EB,HIGH);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
}

void loop()
{
  //right motor
  
  //pwm cuts out at 0  so this is a good channel ;)
  
  analogWrite(IN1,0);
  analogWrite(IN2,255);
  
  
  //left motor
  //pwm starts at 128 instead of zero so need to add this inorder to get speed
  
  analogWrite(IN3,0);
  analogWrite(IN4,255);
  
  delay(2000);
}
