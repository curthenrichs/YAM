//Curt Henrichs
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8

//a specific concrete class that utilizes the drivetrain heigharchy for mecanums


public class Mecanum extends AbstractDriveTrain
{
  //this drivetrain is like tanbk but has the ability to strafe thus it nees its own interpretation of the 
  //rules to which it drives the motors
  
  protected double gyroAngle;
  
  //-------------------------------------<constructors>--------------------------------------
  public Mecanum()
  {
    super();
    gyroAngle = 0;
  }
  
  public Mecanum(Motor[] m, double max, String n,boolean f, boolean c,double initGyroAngle)
  {
   super(m,max,n,f,c);
   gyroAngle = initGyroAngle;
  }
  
  //-------------------------<moifiers>------------------------------------
  public void setAngle(double angle)
  {
    gyroAngle = angle;
  }
  
  //----------------------<acessors>-------------------------------------------
  public double getAngle()
  {
     return gyroAngle; 
  }
  
  //-----------------------<functionality>----------------------------------
  public void drive(double joystickX, double joystickY, double joystickZ)
  {
    //mecanum drivetrain
    //counter variables
    int lc = 0;
    int rc = 0;
    //motor speed variables
    double frontright = joystickY - joystickX - joystickZ;
    double rearright = joystickY + joystickX - joystickZ;
    double frontleft = joystickY + joystickX + joystickZ;
    double rearleft = joystickY - joystickX + joystickZ;
    
    if(motors.length > 4)
    {
      //error
      assert(false);
    }
    else
    {
      for(int i = 0; i < motors.length; i++)
      {
        //if it is left
        if(motors[i].getInvert() == 't')
        {
           //if it is first time the front
           if(lc < 1)
           {
             motors[i].setSpeed(frontleft);
             lc++;
           }
           else
           {
             motors[i].setSpeed(rearleft);
           }
        }
        //else it is right motor
        else
        {
           //if it is first time then front
           if(rc < 1)
           {
             motors[i].setSpeed(frontright);
             rc++;
           }
           else
           {
             motors[i].setSpeed(rearright);
           }
        }
      }
    }
  }
  
  public String toString()
  {
    //get the basics of the drivetrain in string
    String temp = "Name: " + name + " max speed " + maxSpeed + " is it filed centric " + fieldCentric + " is it closed loop " + closedLoop + " gyro angle " + gyroAngle;
    
    temp += " drivetrain motors ";
    
    //get the strings for the motors
    for(int i = 0; i < motors.length; i++)
    {
      temp += motors[i].toString() + " and ";
    }
    
    //return the string
    return temp;
  }
  
}