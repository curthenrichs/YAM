void setup()
{
  Serial.begin(9600);
}

void loop()
{
  char input;
    // if we get a valid byte, read analog ins:
  if(Serial.available())
  {
      input = Serial.read();
      
      if(input == '!')
      {
         input = Serial.read();
         
         if(input == 'M')
         {
         
         }
         else if (input == 'S')
         {
         
         }
         else if (input == 'O')
         {
           
         }
      }
      else if (input == '?')
      {
         input = Serial.read();
         
         if(input == 'S')
         {
            
         }
      }
      else if (input == '#')
      {
        
      }
  }
            
}
