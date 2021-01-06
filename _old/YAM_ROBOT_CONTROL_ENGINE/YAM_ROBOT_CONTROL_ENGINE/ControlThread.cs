using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace YAM_ROBOT_CONTROL_ENGINE
{
    //controls the arduino through serial
    //will receive motorObjects once they are ported to c#
    //and will have all of the sensor classes as well
    //maybe if we get crazy we can bring in the drivetrains as well :)
    public class ControlThread
    {
        //these are the specifics of the robot controller
        private UltraSonic lping = null;
        private UltraSonic mping = null;
        private UltraSonic rping = null;

        private Motor lem = null;
        private Motor rim = null;

        //the kill command value for the thread
        private volatile bool _shouldStop = false;

        public ControlThread()
        {
            //we give them a frowny face :(
        }

        public ControlThread(UltraSonic l, UltraSonic m, UltraSonic r, Motor lm, Motor rm)
        {
            //link our robot to the controller
            lping = l;
            mping = m;
            rping = r;
            lem = lm;
            rim = rm;
        }

        public void DoWork()
        {
            if (!_shouldStop)
            {
                //run the thread
                Console.Out.WriteLine("I am running: controlThread");
            }
            Console.Out.WriteLine("Control Thread ended");
        }

        //the killer of the thread
        public void RequestStop()
        {
            _shouldStop = true;
        }

        private SerialPort setupSerial()
        {
            SerialPort s = null;
            try
            {
                //delcare new serial port
                s = new SerialPort("COM4", 9600);

                //return the port
            }
            catch(System.IO.IOException e)
            {
                Console.Out.WriteLine(e.ToString());
            }
            return s;
        }
    }
}
