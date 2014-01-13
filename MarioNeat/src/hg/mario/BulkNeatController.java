package hg.mario;


import java.util.ArrayList;
import java.util.List;




import org.jgap.Chromosome;

import ch.idsia.agents.Agent;
import ch.idsia.agents.controllers.BasicMarioAIAgent;
import ch.idsia.agents.learning.LargeMLPAgent;
import ch.idsia.benchmark.mario.environments.Environment;
import ch.idsia.evolution.Evolvable;
import ch.idsia.evolution.MLP;

import com.anji.integration.Activator;
import com.anji.integration.ActivatorTranscriber;


public class BulkNeatController extends BasicMarioAIAgent implements Agent{

static private String name = "LargeMLPAgent";

 int numberOfOutputs = Environment.numberOfKeys;
 int numberOfInputs = 28;


private Activator activator;

public BulkNeatController(Activator activator){
    super(name);
	this.activator = activator;
}



public boolean[] getAction()
{
    double[] inputs;// = new double[numberOfInputs];
    byte[][] scene = levelScene;
    inputs = new double[numberOfInputs];
    int which = 0;
//    for (int i = -3; i < 4; i++)
//    {
//        for (int j = -3; j < 4; j++)
//        {
//            inputs[which++] = probe(i, j, scene);
//        }
//    }
//    for (int i = -3; i < 4; i++)
//    {
//        for (int j = -3; j < 4; j++)
//        {
//            inputs[which++] = probe(i, j, enemies);
//        }
//    }
    
    for (int i = -2; i < 3; i++)
    {
        for (int j = -2; j < 3; j++)
        {
            inputs[which++] = probe(i, j, scene);
        }
    }
    inputs[inputs.length - 3] = isMarioOnGround ? 1 : 0;
    inputs[inputs.length - 2] = isMarioAbleToJump ? 1 : 0;
    inputs[inputs.length - 1] = 1;
	
	return convertOutput(activator.next(inputs));		
}

private boolean[] convertOutput(double[] outputs) {
	boolean[] returnVal = new boolean[numberOfOutputs];;
	
	for (int i = 0; i < outputs.length; i++) {
		if (outputs[i] > 0.5) {
			returnVal[i] = true;
		}
	}
	
	return returnVal;
	
//	if (bestIndex == 0) { return MOVE.UP; }
//	if (bestIndex == 1) { return MOVE.RIGHT; }
//	if (bestIndex == 2) { return MOVE.DOWN; }
//	return MOVE.LEFT; // i == 3

}

public String getName()
{
    return name;
}

private double probe(int x, int y, byte[][] scene)
{
    int realX = x + 11;
    int realY = y + 11;
    return (scene[realX][realY] != 0) ? 1 : 0;
}


}
