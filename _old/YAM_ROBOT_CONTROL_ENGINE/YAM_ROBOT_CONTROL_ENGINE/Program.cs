using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace YAM_ROBOT_CONTROL_ENGINE
{
    class Program
    {
        static private UltraSonic lping;
        static private UltraSonic mping;
        static private UltraSonic rping;

        static private Motor lm;
        static private Motor rm;

        //mode 0 is driving auton; mode 1 is teleop; mode  2 is chatbot 
        static private int mode = 0;
        static private bool runServer = true;
        static private bool collisionIminate = false;
        static private bool backupRun = false;

        static void Main(string[] args)
        {
            //create control thread for arduino
            ControlThread cont = new ControlThread();
            Thread controlThread = new Thread(cont.DoWork);

            //Control modules this part is a state machine inside of a loop
            while(runServer)
            {
                //get updates from all the sensors
                //the basic ultrasonics are already automagically updating

                
                //get what the user wants from the UI thread
                if(false) //the false is becasue i dont haz a gui at moment 
                {

                }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                else if(mode == 0)
                {
                    ///------------------------------------
                    ///  this is autonomous driving 
                    ///-------------------------------------
                    
                    //collision
                    if (checkForCollision())
                    {
                        //stop motors
                        lm.stop();
                        rm.stop();
                        //report as collsion val
                        collisionIminate = true;
                    }

//*****************impement behaviors in subsumtion style highest level first**********************
                    
                    //after not backed up check for turn and turn
                    if(backupRun)
                    {
                        if(lping.getDistance() <= rping.getDistance())
                        {
                            //turn right
                            lm.setSpeed(9);
                            rm.setSpeed(0);
                        }
                        else
                        {
                            //turn left
                            lm.setSpeed(0);
                            rm.setSpeed(9);
                        }

                        //delay so an action can occur
                        Thread.Sleep(500);
                    }
                    // back up
                    else if(collisionIminate)
                    {
                        //go backwards or negative
                        lm.setSpeed(-8);
                        rm.setSpeed(-8);

                        Thread.Sleep(500);
                        backupRun = true;
                    }
                    //go forward
                    else if(!collisionIminate)
                    {
                        lm.setSpeed(8);
                        lm.setSpeed(8);

                        Thread.Sleep(200);
                    }
                    //stop
                    else
                    {
                      lm.stop();
                      rm.stop();
                    }
                    
                }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
                else if(mode == 1)
                {
                    ///--------------------------
                    ///this is teleop control
                    ///--------------------------
                }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                else if(mode == 2)
                {
                    ///-------------------------------
                    ///this is the auton chatbot program
                    ///-------------------------------
                }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                else
                {
                    //this is an error case will report to console
                    Console.Out.WriteLine("The mode has not been selected properly. It reported a " + mode);
                }
            }
        }

        static public bool checkForCollision()
        {
            //this i what is checked for distance from an object
            int threshold = 15;

            if (mping.getDistance() <= threshold)
            {
                return true;
            }

            return false;
        }
    }
}
