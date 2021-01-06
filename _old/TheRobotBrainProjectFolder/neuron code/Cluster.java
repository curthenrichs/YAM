import java.util.ArrayList;

//A cluster is a group of layers 
//it allows for the user not make a network of interconnected neurons
//methods used to access the cluster are void .calc() and ArrayList<Double> .getOutput()

public class Cluster
{
  //the list of layers in the cluster
  protected  ArrayList<Layer> layers = new ArrayList<Layer>();
  //the list of final outputs out of the cluster
  protected  ArrayList<Double> output = new ArrayList<Double>();
  
  //defualt constructor dont use
  public Cluster()
  {
    layers = null;
  } 
  
  //builds an array based on the number of layers desired how much of an input size you have 
  //and how many neuorns to start and end with it fills the rest in by itself <This one dos not work corectly yet>
  public Cluster(int numLayers,int inputLayerSize, int startNeuronAmount, int endNeuronAmount)
  {
    //initalize the array
    layers.add(new Layer(inputLayerSize,startNeuronAmount));
    
    //calculate the steps between the neurons
    //provided end neuronAmount is less than start
    int step = (startNeuronAmount - endNeuronAmount)/ numLayers;
    
    //fill in the layers
    for(int i = 1; i < numLayers; i++)
    {
      layers.add(new Layer(startNeuronAmount - step * (i - 1), startNeuronAmount - step * i));
    }
  }
 
  ///*********************************************************************************************
  ///---------------------------------------------------------------------------------------------
  ///     this is the main constructor for the cluster class since it creates all of the stuff
  /// *********************************************************************************************
  ///8888888888888888888888888888888888888888888888888888888888888888888888
  ///----------------------------------------------------------------------------------------------
  public Cluster(ArrayList<Integer> lengthsOfLayers,int inputSize)
  {
    //set first layer
    layers.add(new Layer(inputSize,lengthsOfLayers.get(0)));
    
    //fill in the next layers
    for(int i = 1; i < lengthsOfLayers.size(); i++)
    {
       layers.add(new Layer(lengthsOfLayers.get(i-1),lengthsOfLayers.get(i)));
    }
  }
  
  
  
  //sets the layers in the arraylist if the user wants to change it
  public void setLayers(ArrayList<Layer> l)
  {
    layers = l; 
  }
  
  //gets the layers in the cluster
  public ArrayList<Layer> getLayers()
  {
    return layers; 
  }
  
  //gets the outputs from the cluster
  public ArrayList<Double> getOutput()
  {
    return output; 
  }
  
  //runs the calculation for the cluster and sets the outputs
  public void calc(ArrayList<Double> in)
  {
    //seed inital layer
    layers.get(0).calc(in);
    
    //cascade through array
    for(int i = 1; i < layers.size(); i++)
    {
        layers.get(i).calc(layers.get(i-1).getOutput());
    } 
    
    //take the last layers output as the clusters output
    output = layers.get(layers.size() - 1).getOutput();
  }
  
  public String toString()
  {
    ArrayList<Integer> template = new ArrayList<Integer>();
    for(int i = 0; i < layers.size(); i++)
    {
      template.add(layers.get(i).getNeurons().size());
    }
    //makes a formated string
    String s = String.format("C{" + template.size()+ "}(" + template + ")" + "L("+ layers.get(0).getInputSize() + ")" + "%n" + layers.toString());
    return s;
  }
}