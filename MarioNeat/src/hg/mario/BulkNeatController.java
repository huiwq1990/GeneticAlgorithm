package hg.pacman;

import itu.jgdiejuu.torcs.Action;
import itu.jgdiejuu.torcs.NEATController;

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

import dataRecording.DataTuple;
import pacman.controllers.Controller;
import pacman.game.Game;
import pacman.game.Constants.MOVE;

public class BulkNeatController extends BasicMarioAIAgent implements Agent{

static private String name = "LargeMLPAgent";

final int numberOfOutputs = Environment.numberOfKeys;
final int numberOfInputs = 101;


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
    for (int i = -3; i < 4; i++)
    {
        for (int j = -3; j < 4; j++)
        {
            inputs[which++] = probe(i, j, scene);
        }
    }
    for (int i = -3; i < 4; i++)
    {
        for (int j = -3; j < 4; j++)
        {
            inputs[which++] = probe(i, j, enemies);
        }
    }
    inputs[inputs.length - 3] = isMarioOnGround ? 1 : 0;
    inputs[inputs.length - 2] = isMarioAbleToJump ? 1 : 0;
    inputs[inputs.length - 1] = 1;
	
	return convertOutput(activator.next(inputs));		
}

private boolean[] convertOutput(double[] outputs) {
	
	int bestIndex = -1;
	double bestValue = Double.NEGATIVE_INFINITY;
	
	for (int i = 0; i < outputs.length; i++) {
		if (outputs[i] > bestValue) {
//			bestValue = outputs[i];
			bestValue = clamp(outputs[i],0,1);
			bestIndex = i;
		}
	}
	
	if (bestIndex == 0) { return MOVE.UP; }
	if (bestIndex == 1) { return MOVE.RIGHT; }
	if (bestIndex == 2) { return MOVE.DOWN; }
	return MOVE.LEFT; // i == 3

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
