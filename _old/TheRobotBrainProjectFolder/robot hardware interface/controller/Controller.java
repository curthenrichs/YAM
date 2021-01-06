import java.io.*;
import java.net.*;
import java.util.ArrayList;

//the standard port for the black box serial converter is the 100 on the localhost machine


public class Controller
 {
  //this is a tcp ip socket so it goes to a black box server for serial contol this is the client of that
  protected Socket controllerStream;
  //this is the input stream for the controller
  protected DataInputStream input;
  //the output stream to the controller
  protected PrintStream output;
  //the id of the controller for the communication bus
  protected String deviceID = "";
  
  //--------------------------------------------------------------
  //contructors
  //-------------------------------------------------------------
  public Controller()
  {
   //creates the socket for the subcontroller 
   try
   {
     controllerStream = new Socket("localhost",100);
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
   
   //creates the input stream
   try
   {
     input = new DataInputStream(controllerStream.getInputStream()); 
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
   
   //creates the outputStream
   try
   {
     output = new PrintStream(controllerStream.getOutputStream());
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
  }
  
  //creates the standard controller
  public Controller(String devID)
  {
   //creates the socket for the subcontroller 
   try
   {
     controllerStream = new Socket("localhost",100);
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
   
   //creates the input stream
   try
   {
     input = new DataInputStream(controllerStream.getInputStream()); 
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
   
   //creates the outputStream
   try
   {
     output = new PrintStream(controllerStream.getOutputStream());
   }
   catch(IOException e)
   {
     System.out.println(e); 
   }
   
   //designates device id
   deviceID = devID;
  }
  
  //methods
  //--------------------------------------------------------------------------------------------
  //special end call
  //change this for output
  public void close()
  {
    output.print ("#;");
    try
    {
      output.close();
      input.close();
      controllerStream.close();
    }
    catch(IOException e)
    {
      System.out.println(e); 
    }
  }
  
  //accessors
  public String toString()
  {
   return "!" + deviceID + "," + controllerStream + ";"; 
  }
  
  public String getID()
  {
   return deviceID; 
  }
  
  public String getIn()
  {
    //read the streams next feed back
   String in = "";
   char temp;
   
   do
   {
     try
     {
       temp = input.readChar();
       in = in + temp;
     }
     catch(IOException e)
     {
       System.out.println(e);
       break;
     }
   }while(temp != ';');
          
   return in; 
  }
  
  //modifiers
  public void setId(String id)
  {
   deviceID = id; 
  }
  
  public void setOut(String out)
  {
    output.print(out);
  }
  
  public String help()
  {
   return "this is the help function for the controller. Use this to learn the \ncommands that the serial interface needs. \n! means to do something\n? means to ask for something\n# means to cut all running tasks and shutdown  "; 
  }
  
  public ArrayList<Sensor> translate(ArrayList<Sensor> sensors)
  {
    try
    {
      //while we have sensor values
      while(input.available() > 0)
      {
        //get the message
        String in = getIn(); 
        //translate out the peices of information
        String id = "";
        int value = 0;
        
        //search for the sensor that corresponds to id and set value
        for(int i = 0; i < sensors.size(); i++)
        {
          if(sensors.get(i).getID().equals(id))
          {
            sensors.get(i).setValue(value); 
          }
        }
      }
    }
    catch(IOException e)
    {
      System.out.println(e); 
    }
    return sensors;
  }
 }