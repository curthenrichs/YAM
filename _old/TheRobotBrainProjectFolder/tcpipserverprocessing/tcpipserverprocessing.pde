import processing.serial.*;
import processing.net.*;

Serial myPort;
Server tcpServer;

void setup()
{
  size(200,200);
  //switch to correct port when needed for the other controller
  myPort = new Serial(this,"COM3",9600); 
  
  tcpServer = new Server(this,100);
}

void draw()
{
  try
  {
    //create the current cleint for the program
    Client connection = tcpServer.available();
    //if it ia a real connection
    if(connection != null)
    {
      System.out.println("Server is connected");
    //read from the tcp stream and write to the serial
      while(connection.available() > 0)
      {
        char temp = (char)connection.read();
        myPort.write(temp);
        System.out.println(temp);
      }  
    
      //read from serial and write to tcp
      while(myPort.available() > 0)
      {
        char temp = (char)myPort.read();
        connection.write(temp);
        System.out.println(temp);
      }
    }
  }
  catch(Exception e)
  {
    System.out.println("error in socket"); 
  }
}
