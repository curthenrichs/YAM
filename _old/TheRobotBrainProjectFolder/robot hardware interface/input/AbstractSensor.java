public abstract class AbstractSensor implements Sensor
{
  //this is the device id of the sensor
  protected String devID;
  
  public String serialOutCommand()
   {
    String temp = "";
    temp = "?" + devID + ";";
    return temp;
   }
  
  public String getID()
  {
    return devID;
  }
  
  //modifiers need error trap
   public void setID(String id)
   {   
    devID = id;
   }
   //abstract methods that need to be defined later
   abstract public String toString();
}