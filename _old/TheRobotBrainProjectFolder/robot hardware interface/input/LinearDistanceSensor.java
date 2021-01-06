public class LinearDistanceSensor extends AbstractSensor
{
   //the distance value it represnts
   protected int distance = 0;
   //the max distance it can read saftly
   protected int MAX_dist = 1000;
  
   //constructors
   //default
   public LinearDistanceSensor()
   {
    devID = "";
    distance = 0;
    MAX_dist = 200;
   }
  
   //takes some values
   public LinearDistanceSensor(String ID,int max)
   {
    devID = ID;
    MAX_dist = max;
    distance = 0;
   }
  
   public int getMax()
   {
    return MAX_dist;
   }
  
   public int getDist()
   {
    return distance;
   }
  
   public String toString()
   {
    String outputstring = "!" + devID + "," + MAX_dist + "," + distance + ";";
    return outputstring;
   }
   
   public void setMax(int max)
   {
    MAX_dist = max;
   }
  
   public void setValue(int value)
   {
    distance = value;
   }
  }