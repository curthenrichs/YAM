//Curt Henrichs 
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8


//
//
///  >>>>>>>>>>>>>  for some reason this does not work on my laptop because of compiler so yeah not using right now
//
//


//the organizable interface for the heigharchy

public interface Drivetrain
{
  //the motorsof the drivetrain
   public Motor[] getMotors();
   //name of drivetrain
   public String getName();
   //if it needs a gyro reading to worry about
   public boolean isFieldCentric();
   //if closed loop is on or not
   public boolean closedLoopOn();
   //a to string which can be used for debugging
   public String toString();
   //getMaxSpeed holds the max set speed of the drivetrain
   public double getMaxSpeed();
   
   //sets the values for the motors if they change
   public void setMotors(Motor[] m);
   //sets the name of the drivetrain
   public void setName(String n);
   //sets if it is fied centric or not
   public void fieldCentric(boolean f);
   // set if open or closed loop
   public void closedLoop(boolean c);
   //tells it to drive while this is vopid in this example we may eventually if this were ever really used
   //we would not possibly do it this way and return an array of motor values for the motor
   public void drive(double joystickX, double joystickY, double joystickZ);
   //this sets the max speed
   public void setMaxSpeed(double max);
}