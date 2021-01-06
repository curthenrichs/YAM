//Curt Henrichs 
//AP Computer Science ~ Java
//Mr. Pratt
//Chapter 8

//client for the hiearchy that i created for drivetrains

public class driver
{
   public static void main(String[] args)
   {
     // we have a tank drive a mecanum drive and a swerve drive sytem here.
     
     //this is our max speed
     double maxSpeed = 12;
     //our three "joystick" values
     double joyX = 0;
     double joyY = 5;
     double joyZ = 15;
     //our motor array for drive wheels
     Motor[] motors = new Motor[4];
     motors[0] = new Motor("1", 'f');
     motors[1] = new Motor("2", 't');
     motors[2] = new Motor("3", 'f');
     motors[3] = new Motor("4", 't');
     Motor[] rMotors = new Motor[1];
     rMotors[0] = new Motor("rotation", 'f');
     
     //our drivetrains
     AbstractDriveTrain tank = new Tank("tank drive", motors,maxSpeed);
     AbstractDriveTrain swerve = new Swerve(motors,rMotors,"swerveDrive",maxSpeed);
     AbstractDriveTrain mecan = new Mecanum(motors, maxSpeed,"mecanum drive",false,true,0);
     
     //we now do a little playing with them inorder to show they work.
     tank.drive(joyX,joyY,joyZ);
     swerve.drive(joyX,joyY,joyZ);
     mecan.drive(joyX,joyY,joyZ);
     
     //output the values to the screen to prove everything works.
     System.out.println(tank.toString());
     System.out.println(swerve.toString());
     System.out.println(mecan.toString());
   }
}