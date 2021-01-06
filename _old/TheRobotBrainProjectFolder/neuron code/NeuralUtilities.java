import java.util.Scanner;
import java.util.ArrayList;
import java.io.*;

public class NeuralUtilities
{
    ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ///
  ///  this function loads the network from a text file to create the network based off of the toString() onece i 
  //                                                                                     get it going
  ///
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   public static Cluster load(String inputFile) throws IOException
  {
    //the left index and right index necessary to extract
    int left = 0;
    int right = 0;
    //this holds the number of layers in the arraylist template
    int layerC = 0;
    //this holds the input size for the network
    int inputSize = 0;
    //get a template of  the neuron cluster
    ArrayList<Integer> netTemplate =  new ArrayList<Integer>();
    //file input stream reader for the network
    Scanner in = new Scanner(new File(inputFile));
    String input = in.nextLine();
    
    //this will be the cluster for the network
    Cluster net = new Cluster();
    
    //this explanes how to read the text file:
    /* 
     * in {#} is the number of layers in the cluster and those in ([#]) is the number of neurons in that layer
     *    Then it denotes the layer with an L
     *  Then denotes the neuron with an n
     *  Then denotes the weights of the neurons with a w;  ([#]) denotes input size
     */
    
    //get the number of layers in the cluster
    {
      left = 0;
      while(!(input.substring(left,left+1).equals("{")))
      {
        left++; 
      }
      right = left;
      while(!(input.substring(right,right+1).equals("}")))
      {
        right++;
      }
      //get the subsring 
      String layerCount = input.substring(left+1,right);
      layerC = Integer.parseInt(layerCount);
      //set varaibles for next number
      left = right + 1;
    }
    
    //set up the template
    {
      //get the number of neurons in that layer
      for(int i = 0; i < layerC; i++)
      {
        while(!(input.substring(left,left+1).equals("[")))
        {
          left++;
        }
        right = left;
        while(!(input.substring(right,right+1).equals("]")))
        {
          right++;
        }
        String neuronSize = input.substring(left+1,right);
        int neuron = Integer.parseInt(neuronSize);
        netTemplate.add(neuron);
        //set varaibles for next number
        left = right + 1;
      }
    }
    
    //get the input size for the network
    {
       while(!(input.substring(left,left+1).equals("(")))
        {
          left++;
        }
        right = left;
        while(!(input.substring(right,right+1).equals(")")))
        {
          right++;
        }
        
        String inputS = input.substring(left+1,right);
        inputSize = Integer.parseInt(inputS);
        //set varaibles for next number
        left = right + 1;
    }
    
    //construct the neural network
    net = new Cluster(netTemplate,inputSize);
     
    //set up locat line read variables so that it knows where it is at any given point
    int lastIndex= 0;
    //add in the wheights of the cluster
    //for the number of layers in the array
    for(int t = 0; t < net.getLayers().size();t++)
    {
      //get the next string of data for this layer
      String weightLine = in.nextLine();
      
      //sets the length of the weights size
      int size = 0;
      
      //for the number of neurons
      for(int n = 0; n < net.getLayers().get(t).getNeurons().size();n++)
      {
        //know how many weights to grab
        if(n == 0)
        {
          //find the length of that weight value which is common for all neurons in a layer
          //set some index paramters os we can extract the data
          int l = 3;
          int r = l + 1;
          
          //index right tillwe hit a non integer
          while((weightLine.substring(r,r+1).equals("0")) || (weightLine.substring(r,r+1).equals("1")) || (weightLine.substring(r,r+1).equals("2")) || (weightLine.substring(r,r+1).equals("3")) || (weightLine.substring(r,r+1).equals("4")) || (weightLine.substring(r,r+1).equals("5")) || (weightLine.substring(r,r+1).equals("6")) || (weightLine.substring(r,r+1).equals("7")) || (weightLine.substring(r,r+1).equals("8")) || (weightLine.substring(r,r+1).equals("9")))
          {
            r++;
          }
          
          String s = weightLine.substring(l,r);
          size = Integer.parseInt(s);
          lastIndex = 7;
        }
       
        //creates the arraylist of weights
        ArrayList<Double> weights = new ArrayList<Double>();
        
        //pluck out the weights
         for(int i = 0; i < size;i++)
         {
             //set varaible up
             double w = 0;
             //search weight
             int j = lastIndex;
             String weight = "";
             //this trips when we use scientific form
             boolean trip = false;
             int exponent = 0;
             String exp = "";
             
             //pluck weight out
             while((!(weightLine.substring(j,j+1).equals(",")))&&(!(weightLine.substring(j,j+1).equals("]"))))
             {
               //check to make sure it is a number
               if((weightLine.substring(j,j+1).equals("0"))||(weightLine.substring(j,j+1).equals("1"))||(weightLine.substring(j,j+1).equals("2"))||(weightLine.substring(j,j+1).equals("3"))||(weightLine.substring(j,j+1).equals("4"))||(weightLine.substring(j,j+1).equals("5"))||(weightLine.substring(j,j+1).equals("6"))||(weightLine.substring(j,j+1).equals("7"))||(weightLine.substring(j,j+1).equals("8"))||(weightLine.substring(j,j+1).equals("9"))||(weightLine.substring(j,j+1).equals("."))||(weightLine.substring(j,j+1).equals("-"))||(weightLine.substring(j,j+1).equals("E")))
               {
                 if(weightLine.substring(j,j+1).equals("E"))
                 {
                   //convert out of scientific
                   trip = true;
                 }
                 else if(trip)
                 {
                   exp = exp + weightLine.substring(j,j+1);
                 }
                 else
                 {
                   //we keep the value
                   weight = weight + weightLine.substring(j,j+1);
                 }
               }
               
               j++;
             }
             //get the exponent value for the scientific form
             exponent = Integer.parseInt(exp);
            
             lastIndex = j+1;
             //parse weight
             w= Double.parseDouble(weight);
             
             //add weight
             weights.add(w * Math.pow(10,exponent));
             
             //end of neuron signal
             if(weightLine.substring(j,j+1).equals("]"))
             {
               lastIndex += 3;
             }
             //loop back up
         }
         
         //give that neuron the weights
         net.getLayers().get(t).getNeurons().get(n).setWeights(weights);
      }
    }
    
    //return the network
    return net;
  }
   
     ///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ///
  ///  this function unloads the network to a text file to create the network based off of the toString()
  ///
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   public static void output(Cluster net,Scanner in)throws IOException
   {
     //ask what fiel name it should be saved as
    System.out.println("What would you like to name this file");
    String filename = in.nextLine();
    
    PrintWriter writer = new PrintWriter(new File(filename + ".txt"));
    writer.write(net.toString());
    writer.flush();
    writer.close();
   }
   
}