//this is a test peice of code
//it just echos the serial back from input to output

boolean a = false;
char i = 'g';
void setup()
{
 Serial.begin(9600); 
}

void loop()
{
  if(Serial.available() > 0)
  {
    i = (char)Serial.read();
    
    if(a)
    {
      digitalWrite(13, LOW); 
      a = false;
    }
    else
    {
      digitalWrite(13, HIGH); 
      a = true;
    }
  }
  delay(500);
  Serial.println(i);
}
