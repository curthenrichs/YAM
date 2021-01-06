using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;

//note where i left off in testing, got alot of errors reading back try checking the serial port settings for the arduino plarity and all
//second make this code more robust and give it a kill command so it knows when to release componenets


namespace tcpToSerial
{
    class Program
    {
        static void Main(string[] args)
        {
            //writes all available serial ports
            for (int i = 0; i < SerialPort.GetPortNames().Length; i++)
            {
                Console.WriteLine(SerialPort.GetPortNames()[i]);
            }
            
            //defualt for now will change to specific port for that robot.
            SerialPort serial = new SerialPort("COM4", 9600);
            serial.Parity = 0;
            //this creates a tcp socket on port 100 to localhost
            System.Net.IPAddress ipAd = System.Net.IPAddress.Parse("127.0.0.1");
            System.Net.Sockets.TcpListener tcp = new System.Net.Sockets.TcpListener(ipAd, 100);
            //this defines the client that is connected
            System.Net.Sockets.Socket client;

            //this runs the loop till end command
            bool run = true;
            

            //open the serial port
            try
            {
                serial.Open();
                //output the current state
                Console.WriteLine("Opened Comm port on port " + serial.PortName + " with baud rate " + serial.BaudRate);
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }

            

            //open the tcp server
            tcp.Start();
            //output the current state
            Console.WriteLine("Opened tcp server on port 100 of " + ipAd.ToString());

            //create the client
            client = tcp.AcceptSocket();
            Console.WriteLine("Connected to client " + client.RemoteEndPoint);

            //run an inifinte loop of reading from tcp and writing to serial
            //and reading from serial to write to tcp
            while (run)
            {
                //the input buffer varaible
                if (client.ReceiveBufferSize > 0)
                {
                    byte[] inp = new byte[1];
                    //gets values from the stream and sends them out to serial
                    client.Receive(inp);
                    serial.Write(inp, 0, inp.Length);
                    Console.WriteLine("in from client: " + Convert.ToChar(inp[0]));
                }

                //gets value from serial and outputs it to tcp
                if (serial.BytesToRead > 0)
                {
                    //normal operation
                    byte[] outp = new byte[1];
                    serial.Read(outp, 0, outp.Length);
                    client.Send(outp);
                    Console.WriteLine("out to client: " + Convert.ToChar(outp[0]));
                }
            }
        }
    }
}
