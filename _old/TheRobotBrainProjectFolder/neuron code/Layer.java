import java.util.ArrayList;

//A layer is a group of neurons  in a row that calculates
//layers can calculate their respective neurons based on their input count that is cascaded through them

public class Layer
{
  //holds the neurons assoistated with that layer
  protected ArrayList<Neuron> neurons;
  //outputs of all the nuerons in that layer
  protected ArrayList<Double> outputs = new ArrayList<Double>();
  //holds if this is the inital configuration of the arraylist
  private boolean firstTime = true;
  protected int inputLength = 0;
  
  //defualt do not use ever
  public Layer()
  {
    neurons = null;
  }
  
  //constructor for the nlayer that constructs ana arraylist
  public Layer(int inputSize, int layerSize)
  {
    neurons = new ArrayList<Neuron>();
    for(int i = 0; i < layerSize; i++)
    {
      neurons.add(new Neuron(inputSize,0.5)); 
    }
    
    inputLength = inputSize;
  }
  
  //constructor for the layer class that builds the neuron arraylist
  public Layer(int inputSize, int layerSize, double threshold)
  {
    neurons = new ArrayList<Neuron>();
    for(int i = 0; i < layerSize; i++)
    {
      neurons.add(new Neuron(inputSize,threshold)); 
    }
    
    inputLength = inputSize;
  }
  
  //constructor for the layer class
  public Layer(ArrayList<Neuron> n)
  {
     neurons = n; 
  }
  
  
  //sets the neurons if the user wants to switch them
  public void setNeurons(ArrayList<Neuron> n)
  {
    //set a new layer of neurons
    neurons = n;
    //reset the first time varable
    firstTime =  true;
    //resets the outputs
    outputs = new ArrayList<Double>();
  }
  
  //gets the nueorns if the user wants to switch them
  public ArrayList<Neuron> getNeurons()
  {
    return neurons;
  }
  
  //sends an arraylist of outputs to the cluster
  public ArrayList<Double> getOutput()
  {
    return outputs;
  }
  
  //calculates the neurons activity in the neuron
  public void calc(ArrayList<Double> in)
  { 
    //calculate the neurons for all the things
    for(int i = 0; i < neurons.size(); i++)
    {
       neurons.get(i).act(in);
       
       if(firstTime)
       {
         outputs.add(i,neurons.get(i).getOutput()); 
       }
       else
       {
         outputs.set(i,neurons.get(i).getOutput()); 
       }
    }
    firstTime = false;
  }
  
  //gets the input size for that layer
  public int getInputSize()
  {
     return inputLength; 
  }
  
  //the to string variable
  public String toString()
  {
    String temp = "n("  + neurons.get(0).getWeights().size() + ")" + neurons.toString(); 
     return temp;
  }
}