public interface Sensor
{
  //gets the id of the unit for the controller
  public String getID();
  //sets the id for the sensro
  public void setID(String id);
  //used to output to the console
  public String toString();
  //used to output to the controller
  public String serialOutCommand();
  //the set value command so we dont have to cast
  public void setValue(int value);
}