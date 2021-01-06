//Curt Henrichs
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8

//abstract class for teh drivetrain heigharchy

abstract public class AbstractDriveTrain implements Drivetrain
{
   //variables common to all of them
  protected Motor[] motors;
  protected double maxSpeed;
  protected String name;
  protected boolean fieldCentric;
  protected boolean closedLoop;
  
  //-------------------------------------------------<CONSTUCTORSSSSSSZSSSSSS>----------------------------------------------
  public  AbstractDriveTrain()
  {
    //this one should never ever ever be called and if it is i will punish people
    
    //there needs to be at least two motors in a drivetrain
    motors = new Motor[2];
    //this is the scaler by which we output our stuff to
    maxSpeed = 0;
    //name of drivetrain
    name = "";
    //if it is field or robot oriented
    fieldCentric = false;
    //if it has a closed loop component to it
    closedLoop = false;
  }
  
  public AbstractDriveTrain(Motor[] m, double max, String n)
  {
    //motr stuff
    motors = m;
    //out max speed will be 12 if we go on a voltge scale
    maxSpeed = max;
    //name of it
    name = n;
    
    //they neglected to mention these arguments so they are defaulted
    fieldCentric = false;
    closedLoop = false;
  }
  
  public AbstractDriveTrain(Motor[] m, double max, String n,boolean f, boolean c)
  {
    //motr stuff
    motors = m;
    //out max speed will be 12 if we go on a voltge scale
    maxSpeed = max;
    //name of it
    name = n;
    //this is if field or not
    fieldCentric = f;
    //closed loop or not
    closedLoop = c;
  }
  
  //++++++++++++++++++++++++++++++++++++<GETTERS>++++++++++++++++++++++++++++++++++++++++
  
  
  //this gives the refeence to all of the motors used
   public  final Motor[] getMotors()
   {
     return motors;
   }
   
   //this givest the name of the drivetrain
   public final String getName()
   {
     return name;
   }
   
   //if it is field centric
   public final boolean isFieldCentric()
   {
     return fieldCentric;
   }
   
   //closed loop on or off
   public final boolean closedLoopOn()
   {
     return closedLoop;
   }
   
   //generic to stringer
   public  String toString()
   {
     String temp = "";
     
     for(int i = 0; i < motors.length; i++)
     {
        temp += motors[i].getName() + "   " + motors[i].getSpeed() + "   " + motors[i].getInvert() +  "   "; 
     }
     
     temp +="Name of drivetrain  " + name + "   maxspeed   " + maxSpeed + "     fieldcentric   " + fieldCentric + "   closed loop   " + closedLoop;
     
     return temp;
   }
   
   //this is the max speed of drivetrain
   public final double getMaxSpeed()
   {
     return maxSpeed;
   }
   
   //+++++++++++++++++++++++++++++++++<SETTERS>+++++++++++++++++++++++++++++++++++++++++++
   
   //if we want to reset motors
   public void setMotors(Motor [] m)
   {
     motors = m;
   }
   
   //we want to send the name of the drivetrain
   public final void setName(String n)
   {
     name = n;
   }
   
   //is it field centric
   public final void fieldCentric(boolean f)
   {
     fieldCentric = f;
   }
   
   //is it in closed loop
   public final void closedLoop(boolean c)
   {
     closedLoop = c;
   }
   
   //set the max speed that these motors can go
   public final void setMaxSpeed(double m)
   {
     maxSpeed = m;
   }
   
   //+++++++++++++++++++++++++++++  <Functionality>+++++++++++++++++++++++++++++++++++++++++++++
   
    //this is where they will put their specificik math in here
   public abstract void drive(double joystickX, double joystickY, double joystickZ);
}