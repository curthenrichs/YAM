//Curt Henrichs
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8

//a specific concrete class that utilizes the drivetrain heigharchy for Swerve



public class Swerve extends Tank
{ 
  //these are the motors for that rotate the drivemtors
  protected Motor[] rotationMotors;
  
  //default as it should never ever ever BE USED!!!!!!!!!!!!!!!!
  public Swerve()
  {
     super(); 
     rotationMotors = new Motor[1];
  }
  
  //the constructor we use
  public Swerve(Motor[] dm, Motor[] rm, String n, double max)
  {
    super(n,dm,max);
    rotationMotors = rm;
  }
  
  //sets the rotation motors
  public void setRotationMotors(Motor[] rm)
  {
    rotationMotors = rm;
  }
  
  //gets the rotation motor array back out
  public Motor[] getRotationMotors()
  {
     return rotationMotors; 
  }
  
  //functionality IE the drivetrain
  public void drive(double joystickX, double joystickY, double joystickZ)
  {
    //we use tank for the drivemotors but we  also need to update the rotation motors
    super.drive(joystickX,joystickY,joystickZ);
    
    //get the rate change desired
    double rot = joystickZ;
    
    //calculate the rotaion speed
    if(Math.abs(rot) > super.getMaxSpeed())
    {
       rot = super.getMaxSpeed() * (Math.abs(rot)/rot); 
    }
    
    //set teh outputs
    for(int f = 0; f < rotationMotors.length; f++)
    {
        rotationMotors[f].setSpeed(rot); 
    }
  }
  
  public String toString()
  {
    //get the basics of the drivetrain in string
    String temp = "Name: " + name + " max speed " + maxSpeed + " is it field centric " + fieldCentric + " is it closed loop " + closedLoop + " ";
    
    temp += " drivetrain motors ";
    
    //get the strings for the drive motors
    for(int i = 0; i < motors.length; i++)
    {
      temp += motors[i].toString() + " and ";
    }
    
    temp += "  rotation Motors ";
    
    //get the strings for the rotation motors
    for(int i = 0; i < rotationMotors.length; i++)
    {
      temp += rotationMotors[i].toString() + " and ";
    }
    
    return temp;
  }
}