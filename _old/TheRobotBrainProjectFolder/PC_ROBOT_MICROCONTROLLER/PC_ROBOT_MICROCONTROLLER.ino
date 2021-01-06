
//=============================================<Constants, Global Variables, & Type Defs>==============================================

//global variables that will hold interprogram communication

//holds the thread select value
boolean select = true;
//holds a temp value comming in from the Serial
char temp;

//Motor speed values

//holds the specific speed of the left motor value
double driveML = 0;
//holds the specific speed of the right motor value
double driveMR = 0;

//holds the integer value of the left ping
int pingL = 0;
//holds the integer value for the right ping
int pingR = 0;
//holds the integer value of the middle ping
int pingM = 0;

//constants
//OUTPUTs

//drivetrain
//ID for the left motor
const char DRIVEM_L = '1';
//ID for the right motot
const char DRIVEM_R = '2';

//INPUTS
//ultrasonics
//the ID of the left ping
const char PING_L = '0';
//the ID of the middle ping
const char PING_M = '1';
//the ID value of the middle ping
const char PING_R = '2';

//========================================================<Setup>=============================================================================

void setup()
{
  //sets up the serial port
  Serial.begin(9600);
}

//========================================================<MAIN>================================================================================

void loop()
{
  
  ///<summary>
  /// Tis is the "thread" that activates all motors 
  ///and also gets input for all sensors
  ///<summary>
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(select) //if thte select varaible is true
  {
     //all hardware set here
     
     //get values for all sensors
    
     //get the distance from sultrasonics and update to buffer
     pingL = getdist(PING_L);
     pingM = getdist(PING_M);
     pingR = getdist(PING_R);
     
     //get all switch values
       //which are non at this time 
    
     //set all motors and ouputs to what ever value they are commanded to be at
     //set the left motor
     setMotor(DRIVEM_L,driveML);
     //set the right motor
     setMotor(DRIVEM_R,driveMR);

     //switch select variable to read from serial
     select = false;  
  }
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  ///<summary>
  /// This is the "thread" thaht controls all of the serial data on the robot. This is what communicates to the PC
  ///<summary>
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  else
  {
     //do the serial parsing
     
     //if the serial port has new information
     if(Serial.available())
     {
         //get the value in char form from the serial buffer
         char command = serialReader();
         
         
         //if it is a controller command
         //######################################################################################################
         if (command == '#')
         {
           //read what type of controller command
           char type = serialReader();
           
           //if it is a abort program for safty
           if(type == 'x')
           {
             driveML = 0;
             driveMR = 0;
           }
           //if it is a Give ID of all the controlls. needs work to be done
           else if(type == 'I')
           {
              //serial out all ids with ,
              String  out = "!" + DRIVEM_L;
                      out +=",";
                      out += DRIVEM_R;
                      out += "?";
                      out += PING_L;
                      out += ",";
                      out += PING_M;
                      out += ",";
                      out + PING_R;
                      out += ";";
              Serial.print(out);
           }
           //we got close signal
           else
           {
             driveML = 0;
             driveMR = 0;
           }
         //###########################################################################################################
         }
         //else if it is an exicutable (OUTPUT) command
         //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         else if (command == '!')
         {
           //get command type
           char type = serialReader();
           
           //if it is a dc motor
           if(type == 'M')
           {
             //get ID
             char id = serialReader();
             
             //get the comma
             temp = serialReader();
             //get the invert state of the motor
             char invert = serialReader();
             temp = serialReader();
             //the speed from 0 - 9
             char sped = serialReader();
             //semicolon catch
             temp = serialReader();
               
             int neg = 0;
               
             if(invert == 'f')
             {
               neg = 1;
             }
             else
             {
               neg = -1; 
             }
               
             //set the values as proportion of 256
             int v = calcSpeed(sped) * 256 * neg;
             
             //set to id
             if(id == DRIVEM_L)
             {
               driveML = v;
             }           
             else if(id == DRIVEM_R)
             {
                driveMR = v;
             }
             
           }
           //else if it is a servo... needs to be completed at different date
           else if(type == 'S')
           {
             char id = serialReader();
           }
           
           /*
            *
            *      Add other ouput types here
            *
            *
            */ 
         }
         //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         
         //else if it is a query type
         //????????????????????????????????????????????????????????????????????????????????????????????????
         else if (command == '?')
         {
           //get the sensor in question
           char type = serialReader();
           
           //if it is a linear distance sensor 
           if(type == 'l')
           {
             char id = serialReader();
             
             //get which of the three pings it is
             if(id == '0')
             {
               Serial.write(pingL);
             }
             else if(id == '1')
             {
               Serial.write(pingM);
             }
             else if(id == '2')
             {
               Serial.write(pingR);
             }
           }
           //if it is a switch
           else if(type == 's')
           {
             //we hAVE none at the moment
           }
           
           /*
            *
            *    Add more sensors here as they become available
            *
            *
            */
           
         }
     }
     //???????????????????????????????????????????????????????????????????????????????????????????????
     
     //switch to the operation thread
     select = true;
  }
  //small delay for update
  delay(500);
}

//============================================<Functions>============================================================

int getdist(char id)
{
  
}

//-----------------------------------------------------------------------------------------------------------------
void setMotor(char id,int sp)
{
  
}


//-------------------------------------------------------------------------------------------------------------------
int calcSpeed(char sp)
{
  int holder;
  
  switch(sp)
  {
    case '0':  holder = 0;
               break;
               
    case '1':  holder = 1;
               break;
               
    case '2':  holder = 2;
               break;
               
    case '3':  holder = 3;
               break;
               
    case '4':  holder = 4;
               break;
               
    case '5':  holder = 5;
               break;
               
    case '6':  holder = 6;
               break;
               
    case '7':  holder = 7;
               break;
               
    case '8':  holder = 8;
               break;
               
    case '9':  holder = 9;
               break;
               
    default :  holder = 0;
  }
  
  return holder;
}

//----------------------------------------------------------------------------------------------
char serialReader()
{
  return Serial.read(); 
  
}
