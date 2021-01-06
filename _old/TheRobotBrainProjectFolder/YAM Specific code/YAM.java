//this is the main for yam the robot has three ultrasonic sensors  left middle and right
//it has two motors in a tank drive fashion

import java.util.ArrayList;


public class YAM
{
  private static final double MIN_FRONT_DISTANCE = 25;
  
  public static void main(String[] args)
  {
    //varaibles for life
    boolean alive = true;
    
    ///-----------------------------------------------------------
    ///  the definitions of the Sensors
    ///-----------------------------------------------------------
    Sensor leftUltraSonic = new LinearDistanceSensor();
    Sensor centerUltraSonic = new LinearDistanceSensor();
    Sensor rightUltraSonic = new LinearDistanceSensor();
    //the arrayList is defined from left to right
    ArrayList<Sensor> sensors = new ArrayList<Sensor>();
    sensors.add(leftUltraSonic);
    sensors.add(centerUltraSonic);
    sensors.add(rightUltraSonic);
    
    ///-----------------------------------------------------------
    ///  definition of the Outputs
    ///-----------------------------------------------------------
    Motor leftMotor = new Motor();
    Motor rightMotor = new Motor();
    //the arraylist is defined from left to right]
    ArrayList<Motor> motors = new ArrayList<Motor>();
    motors.add(leftMotor);
    motors.add(rightMotor);
    
    ///----------------------------------------------------------
    ///  the definition of the Sub controller
    ///----------------------------------------------------------
    Controller subProcessor = new Controller();
    
    
    
    //this is the main root code in here we define the behavior in this case i am writing a basic wall banger method
    //and will call that
    //poll sensors 
    
    
    //delay for a time inorder for an update from the controller
    try
    {
      Thread.sleep(1000);
    }
    catch(InterruptedException ex)
    {
      Thread.currentThread().interrupt();
    }
    
    
    while(alive)
    {
      //translate the inputs to ther objects
      subProcessor.translate(sensors);
      
      
      
      //this is the behavior of the robot here
      //process and output
      wallBanger(subProcessor,motors,sensors);
      
      
      
      //send request for new sensor values
      for(int i = 0; i < sensors.size(); i++)
      {
        sensors.get(i).serialOutCommand(); 
      }
      
      //delay for sensor update
      try
      {
        Thread.sleep(1000);
      }
      catch(InterruptedException ex)
      {
        Thread.currentThread().interrupt();
      }
    }
  }
  
  
  
  //Function: wallBanger
  //Purpose: this is a routine that avoids walls and randomly drives around
  //
  //Input: Controller for the sub controller Arraylist of motors and ArrayList of ultrasonic sensors
  //Output: sets the objects values so that it reflects the current state of the system
  
  public static void wallBanger(Controller arduino, ArrayList<Motor> motors,ArrayList<Sensor> ultraSonics)
  {
    final double LEFT_THRESHOLD = 0;
    final double RIGHT_THRESHOLD = 0;
    
    //if our safe distance value for the robot in the forward direction is less than what we see we are fine
    if(((LinearDistanceSensor)ultraSonics.get(1)).getDist() > MIN_FRONT_DISTANCE)
    {
      //set the speed (0 - 9) 
      forward(7,motors,arduino);
    }
    //else we stop and change direction
    else
    {
      //decide if we turn left or if we turn right
      //if both are greater than the threshold go to the larger one
      if((((LinearDistanceSensor)ultraSonics.get(0)).getDist() > LEFT_THRESHOLD) && ((LinearDistanceSensor)ultraSonics.get(2)).getDist() > RIGHT_THRESHOLD)
      {
        if(((LinearDistanceSensor)ultraSonics.get(0)).getDist() > ((LinearDistanceSensor)ultraSonics.get(2)).getDist())
        {
          //turn left
          turn(-90,motors,arduino);
        }
        else
        {
          //turn right
          turn(90,motors,arduino);
        }
      }
      else if(((LinearDistanceSensor)ultraSonics.get(0)).getDist() > LEFT_THRESHOLD)
      {
        //turn left
        turn(-90,motors,arduino);
      }
      else if (((LinearDistanceSensor)ultraSonics.get(2)).getDist() > RIGHT_THRESHOLD)
      {
        //turn right
        turn(90,motors,arduino);
      }
      else
      {
        //turn 180
        turn(180,motors,arduino);
      }
    }
  }
  
  public static void forward(int speed, ArrayList<Motor> motors, Controller arduino)
  {
    //set speed
    motors.get(0).setSpeed(speed);
    motors.get(1).setSpeed(speed);
    //set direction
    motors.get(0).setInvert('f');
    motors.get(1).setInvert('f');
    
    //update
    arduino.setOut(motors.get(0).serialString());
    arduino.setOut(motors.get(1).serialString());
  }
  
  public static void backward(int speed, ArrayList<Motor> motors, Controller arduino)
  {
    //set speed
    motors.get(0).setSpeed(speed);
    motors.get(1).setSpeed(speed);
    //set direction
    motors.get(0).setInvert('t');
    motors.get(1).setInvert('t');
    
    //updATE
    arduino.setOut(motors.get(0).serialString());
    arduino.setOut(motors.get(1).serialString());
  }
  
  public static void turn(int rotation, ArrayList<Motor> motors, Controller arduino)
  {
    final double DELAY_PER_DEGREE = 90;
    
    //set the command
    //set speed
    motors.get(0).setSpeed(4);
    motors.get(1).setSpeed(4);
    //set direction
    if(rotation < 0)  //left
    {
      motors.get(0).setInvert('t');
      motors.get(1).setInvert('f');
    }
    else
    {
       motors.get(0).setInvert('f');
       motors.get(1).setInvert('t');
    }
    
    //output to the controller
    arduino.setOut(motors.get(0).serialString());
    arduino.setOut(motors.get(1).serialString());
    
    //the timed statement to tell it to drive a certain way
    for(int t = 0; t < DELAY_PER_DEGREE * Math.abs(rotation); t++)
    {
      //empty delay loop here
    }
  }
}