//basic digital input

public class IOswitch extends AbstractSensor
{
  //this 
   protected int state;
  
   public IOswitch()
   {
    state = 0;
    devID = "";
   }
  
   public IOswitch(String id)
   {
    state = 0;
    devID = id;
   }
  
   public String toString()
   {
    return state + "," + devID + "," + ";";
   }
  
   public int getState()
   {
    return state;
   }
   
   public void setValue(int value)
   {
     state = value; 
   }
  }