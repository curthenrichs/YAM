using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace YAM_ROBOT_CONTROL_ENGINE
{
    //this thread has the ability to interact with the user hopefully will allow for an epic gui
    public class GUIThread
    {
        private volatile bool _shouldStop = false;

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
    }
}
