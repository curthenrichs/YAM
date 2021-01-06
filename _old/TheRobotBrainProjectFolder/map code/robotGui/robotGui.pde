import processing.net.*;
int port = 1002;
Server myServer;

//this is robot gui the output for the robot code
char[][] map = new char[70][70];

String mapName = "";
String previousMap = "";
int robotX = 0;
int robotY = 0;
String Ping1 = null,Ping2 = null,Ping3 = null;
int GoalX = 0;
int GoalY = 0;
char junk;
char tem = '0';

String[] lines;
PFont f;

int q = 0;

void setup()
{
  size(561,700);
  
  f = createFont("Georgia",12);
  textFont(f);
  
  myServer = new Server(this,port);
}

void draw()
{
  background(235);
  
   //get map updates if necessary
   Client thisClient = myServer.available();
   
   if(thisClient != null)
   {
      if(thisClient.available() > 0)
      {
        //typical transmission of data to the sever will include <map name>;<robotX>;<robotY>;<IF a 1 then goal else IF 0 then no goal>;<Ping1>;<Ping2>;<Ping3>;<GoalX(if applic)>;<GoalY(if applic)>;
        String temp = thisClient.readStringUntil(';');
        mapName = "";
        while(temp.charAt(q) != ';')
        {
          mapName += temp.charAt(q);
          q++; 
        }
        q = 0;
        
        robotX = thisClient.read();
        junk = thisClient.readChar();
        robotY = thisClient.read();
        junk = thisClient.readChar();
        tem = thisClient.readChar();
        junk= thisClient.readChar();
        Ping1 = thisClient.readStringUntil(';');
        Ping2 = thisClient.readStringUntil(';');
        Ping3 = thisClient.readStringUntil(';');
        if(tem == '1')
        {
            GoalX = thisClient.read();
            junk = thisClient.readChar();
            GoalY = thisClient.read();
            junk = thisClient.readChar();
        }
        
      } 
   }
   
   if(mapName.equals(""))
   {
     
   }
   else
   {
     //get map data from txt file
     previousMap = mapName;
     lines = loadStrings("RobotMaps\\" + mapName + "\\" + mapName + ".txt");
   
      for(int x = 0; x < 70; x++)
      {
         String holder = lines[x];
         for(int y = 0; y < 70; y++)
         {
            map[y][x] = holder.charAt(y);
         } 
       }
     
     
     map[robotX][robotY] = 'r';
   
     if(tem == '1')
     {
       map[GoalX][GoalY] = 'g'; 
     }
   }
  
   //display output
//------------------------<read from map and display>---------
   for(int x = 0; x < 70 * 8;x =  x + 8)
   {
     for(int y = 0; y < 70 * 8; y = y + 8)
     {
       char t = map[x/8][y/8];
       
       if(t == 'c')
       {
         fill(0);
       }
       else if(t == 'v')
       {
         fill(75,7,58);
       }
       else if(t == 'd')
       {
         fill(82,61,40);
       }
       else if(t == 'o')
       {
         fill(255);
       }
       else if(t == 'u')
       {
         fill(247,247,20);
       }
       else if(t == 'g')
       {
         fill(35,193,16);
       }
       else if(t == 'r')
       {
         fill(250,5,9);
       }
       else 
       {
         fill(255);
       }
       
        rect(x,y,8,8); 
     }
   }
   
   //sensor data based on last known/ given reading
   fill(0);
   text("Map Name: " + mapName,1,580);
   
   //col one
   text("Robot X: " + robotX, 10, 600);
   text("Robot Y: " + robotY, 10, 616);
   
   text("Goal assigned: " + tem, 10,638);
   
   text("Goal X  : " + GoalX, 10, 638 + 20);
   text("Goal Y  : " + GoalY, 10, 638 + 32);

   //col two
   text("Left Ultrasonic       : " + Ping1, 200,600);
   text("Center Ultrasonic : " + Ping2, 200,616);
   text("Right Ultrasonic   : " + Ping3, 200,632);

}
