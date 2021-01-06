//this is the sensor driver to test to see if every thing is working as it should

public class driver 
{
  public static void main(String[] args)
  {
   //this will test the code for all of the inputs so that we can start feeling more comfortable with them; 
    AbstractSensor nullBump = new IOswitch();
    AbstractSensor nullDist = new LinearDistanceSensor();
    Sensor bump = new IOswitch("0");
    Sensor ping = new LinearDistanceSensor("7",200);
    
    System.out.println(nullBump.toString() + "    " + nullBump.serialOutCommand());
    System.out.println(nullDist.toString() + "    " + nullDist.serialOutCommand());
    System.out.println(bump.toString() + "    " + bump.serialOutCommand());
    System.out.println(ping.toString() + "    " + ping.serialOutCommand());
  }
}