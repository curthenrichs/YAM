//Curt Henrichs
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8

//this is used as a motor controller or a motor concept for the drivetrain

public class Motor 
{
  //holds the name of the motor in this case not very important but we would like to name them something for debugging
  String name;
  //holds the speed for the motor
  double speed;
  //sets if the motor is inverted
  char invert;
  
  //------------------------------------------------------------------------------------------
  //dont use this one!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  public Motor()
  {
    name = "";
    speed = 0;
    invert = 'f';
  }
  
  //the motor constructor that sets the name and if it is inverted (left)
  public Motor(String n, char in)
  {
     name = n;
     invert = in;
     speed = 0;
  }
  
//-------------------------------------------------------------------------------------------  
  //gets the name of the motor
  public String getName()
  {
    return name;
  }
  
  //gets the speed of the motor
  public double getSpeed()
  {
    return speed;
  }
  
  //gets teh invertion of the motor
  public char getInvert()
  {
    return invert;
  }
  
//-------------------------------------------------------------------------------------------
  //sets the name of the motor
  public void setID(String nm)
  {
    name = nm;
  }
  
  //sets the speed of the motors
  public void setSpeed(double spd)
  {
    speed = spd;
  }
  
  //sets the invertion of the motors
  public void setInvert(char in)
  {
    invert = in;
  }
  
  public String toString()
  {
    //get the string to serial ready
   String output = "!" + name + "," + invert + "," + speed + ";";
   return output;
  }
  
  public String serialString()
  {
    //decide if it is negaticeor not in the speed
    char neg = 'f';
    if(invert == 'f')
    {
      if(speed < 0)
      {
        neg = 't'; 
      }
      else
      {
        neg = 'f'; 
      }
    }
    else
    {
      if(speed < 0)
      {
        neg = 'f';
      }
      else
      { 
        neg = 't';
      }
    }

   //get the string to serial ready
   String output = "!" + name + "," + neg + "," + Math.abs(speed) + ";";
   return output;
  }
}