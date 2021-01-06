import processing.net.*;

Client c;
Byte robotX = 0;
Byte robotY = 0;
Byte GoalX = 0;
Byte GoalY = 0;

void setup()
{
  size(200,200);
  c = new Client(this, "LocalHost",1002); 
}

void draw()
{ 
  
  c.write("FamilyRoom;" + char(robotX) + ";" + char(robotY) + ";" + "0;"+ "12;12;13;");
}
