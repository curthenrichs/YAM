//this code is strated both as an avenue to experiemtn with neurons but also to develop the technology for more advanced robot vision system
//the goal is to crreate a neural network system that is composed of clusters of neurons each able to contain its own fitness system
//these clusters are tied togetehr with inputs and outputs set to a random number of values


import java.util.Scanner;
import java.util.ArrayList;
import java.io.*;

public class NeuronMain
{
  public static void main(String[] args) throws IOException
  {
    //          utility varaibles 
    //the number of neurons in the cluster
    int neuroCount = 4;
    //input stream 
    Scanner in = new Scanner(System.in);
    //index locator for output
    int i = 0;
    //this is the load neuron varaible
    boolean load = false;
    //this is the run neuron verse train neuron varaible
    boolean run = false;
    //this boolean runs the trainer when false it ends
    boolean programRun = true;
    
    //    neuron varaibles
    //get a template of  the neuron cluster
    ArrayList<Integer> netTemplate =  new ArrayList<Integer>();
    netTemplate.add(neuroCount);
    //the input index for the network
    ArrayList<Double> testVal = new ArrayList<Double>();
    //the network of the array
   Cluster net ;
    
    
    ///---------------------------------------------------------------------------------
    ///              main code here
    ///------------------------------------------------------------
    
   
   
     //ask user if they would like to load the network or make a new one
    System.out.print("Would you like to load the network from file or create a new one? ");
    String temp = in.nextLine();
    if(temp.equals("load"))
      load = true;
    
    if(load)
    {
      //load from file given
      
      //ask for file name
      System.out.println("What input file would you like to load the network from");
      String inputStream = in.nextLine();
      //oad file
      try
      {
        net = NeuralUtilities.load(inputStream+ ".txt");
      }
      catch(IOException e)
      {
        System.out.println("There is No SPOON!");
        net = null;
      }
    }
    else
    {
      //load from template  >>>>>>>>>>>>>>>>>>>>  eventually add a template buildern so we can set up a network robustly
      //network built with a neural net from the integer arraylist template built and an input index of 3
      net = new Cluster(netTemplate,7);
    }
    
    
      //run the menu
     //change to a switchable  when working so we can save the neuron based on name
    while(programRun)
    {
      System.out.println(net.toString());
      
      //ask if they want to train or if they want to just run it
      System.out.println("Would your like to run the network or train it? ");
      String memp = in.nextLine();
      if(memp.equals("run"))
      {
         run = true;
      }
      else
      {
         run = false;
      }
      
      //running the networ as a finished program
      if(run)
      {
          //run the network
         //input from the user
        testVal = input(in);
        //calculate network
        net.calc(testVal);
        //make an array of the outputs
        ArrayList<Double> output =  net.getOutput();
        //finds the first activated neuron
        for(i = 0; i < output.size(); i++)
        {
          if(output.get(i) == 1)
          {
            break;
          }
        }
        //prints the fruit or veggie that it is
        outObj(i);
      }
      //running the network in trainign mode
      else
      {
        //train the network
        //input from the user
        testVal = input(in);
        //calculate network
        net.calc(testVal);
        //make an array of the outputs
        ArrayList<Double> output =  net.getOutput();
        //outputs the array of outputs of the neurons
        System.out.println("the outputs of the neurons are " + output);
        //finds the first activated neuron
        for(i = 0; i < output.size(); i++)
        {
          if(output.get(i) == 1)
          {
            System.out.println("At index " + i);            // this is for debugging purposes to make sure it fires
            break;
          }
        }
        //prints the fruit or veggie that it is
        outObj(i);
        //train the network
        train(in,testVal,net);
      } 
      
      //ask if the user would like to continue or not
      System.out.println("Would you like to continue? (Y,N)");
      String inp = in.nextLine();
      
      if(inp.equals("y"))
      {
        programRun =  true;
      }
      else
      {
         programRun = false;
      }
    } 
    
    //output to text file
    NeuralUtilities.output(net,in);
  }

  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ///
  ///  this is the input for the user to enter in what the object looks like so the program can figure it out
  ///
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  public static ArrayList<Double>  input(Scanner in)
  {
    //this is the input stream for the network
    ArrayList<Double> netOut = new ArrayList<Double>();
     //this is the input varaible
     String input = "";
    
     //input stuff
     
     //colors select
     System.out.println("Please describe your object accurately ");
     System.out.print("Is your object Blue Green or Yellow: (B,G,Y) ");
     input = in.nextLine();
     
     //set inputs
     if(input.equals("y"))
     {
        netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     if(input.equals("g"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     if(input.equals("b"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     
     //long or skinny
     System.out.print("Is your object long or short: (L, S) ");
     input = in.nextLine();
     
     //sets inputs
     if(input.equals("l"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     if(input.equals("s"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     
     
     //wide or skinny    
     System.out.print("IS your object wide or skinny: (W, S) ");
     input = in.nextLine(); 

     //sets inputs
     if(input.equals("w"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     if(input.equals("s"))
     {
       netOut.add(1.0);
     }
     else
     {
       netOut.add(0.0);
     }
     
     return netOut;
  }
  
  ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ///
  ///    this prints the to the screen what the net says it needs
  ///
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  public static void outObj(int i)
  {
      //guesses which fruti or veggie it is
      switch(i)
      {
        case 0:    System.out.println("Blue Berry");
                        break;
        case 1:     System.out.println("Banana");
                        break;
        case 2:    System.out.println("Cucumber");
                        break;
        case 3:    System.out.println("WaterMelon");
                        break;
        default:   System.out.println("Inconclusive");
                        break;
        
      }
  }
  
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ///
  ///    this implents the training of the neuron if necessary
  ///
  ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  public static void train(Scanner in,ArrayList<Double> input, Cluster net)
  {
    //set the varaibles
    //get the nueorns for the ntwork
    ArrayList<Neuron> neurons = net.getLayers().get(0).getNeurons();
    //the desired index of outputs
    ArrayList<Double> desired = new ArrayList<Double>();
    for(int p = 0; p< neurons.size(); p++)
    {
      desired.add(0.0); 
    }
    
    //check what is correct
    
    //ask if it is correct
      System.out.println("What is the correct answer?");
      String object = in.nextLine();
      int indexOfCorrect = -1;
      
      //check against the fruits we have
      //data bse of fruits
      if(object.equals("blue berry"))
      {
        indexOfCorrect = 0;
      }
      else if(object.equals("banana"))
      {
        indexOfCorrect = 1;
      }
      else if(object.equals("cucumber"))
      {
        indexOfCorrect = 2;
      }
      else if(object.equals("water melon"))
      {
        indexOfCorrect = 3;
      }
      
      //do calculation of the network
      //do weight calculateion
      if(indexOfCorrect == -1)
      {
         //we entered bad data 
      }
      else 
      {
          //set the active desired value
         desired.set(indexOfCorrect,1.0);
        
          
         //this implents the learnign rule
         learningRule(neurons,input,desired);
       }
  }
  
  
  ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// 
  ///   implements the learning rule for the network this can be modified as is necessary
  ///                                              
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  public static void learningRule(ArrayList<Neuron> n,ArrayList<Double> in, ArrayList<Double> des)
  {
    final double rate = 0.4;
    
    for(int i = 0; i < n.size(); i++)
    {
         //if desired = actual
        if(n.get(i).getOutput() == des.get(i))
        {
          //reset weights to same just for clarity
        }
        //if desired is 0 and it fired
        else if(n.get(i).getOutput() > des.get(i))
        {
          //subtract the positive inputs
           for(int k = 0; k < in.size(); k++)
          {
              //if this input signal was high then we positivly reneforcrce
              if(in.get(k) == 1)
              {
                //get the weight assotiated with that input
                double temp = n.get(i).getWeights().get(k);
                //add in the learnig constant
                temp = temp - rate;
                //set the weight to the new number
                n.get(i).getWeights().set(k,temp);
              }
              else  //we negativly reneforce
              {
                //get the weight assotiated with that input
                double temp = n.get(i).getWeights().get(k);
                //subtarct in the learnig constant
                temp = temp + rate;
                //set the weight to the new number
                n.get(i).getWeights().set(k,temp);
              }
          }
        }
        //if the desired is 1 and it was 0
        else
        {
           //add the activted weights since 
          for(int k = 0; k < in.size(); k++)
          {
              //if this input signal was high then we positivly reneforcrce
              if(in.get(k) == 1)
              {
                //get the weight assotiated with that input
                double temp = n.get(i).getWeights().get(k);
                //add in the learnig constant
                temp = temp + rate;
                //set the weight to the new number
                n.get(i).getWeights().set(k,temp);
              }
              else  //we negativly reneforce
              {
                //get the weight assotiated with that input
                double temp = n.get(i).getWeights().get(k);
                //subtarct in the learnig constant
                temp = temp - rate;
                //set the weight to the new number
                n.get(i).getWeights().set(k,temp);
              }
          }
        }
    }
  }
}