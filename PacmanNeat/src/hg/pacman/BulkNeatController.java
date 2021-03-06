package hg.pacman;

import com.anji.integration.Activator;

import dataRecording.DataTuple;
import pacman.controllers.Controller;
import pacman.game.Game;
import pacman.game.Constants.MOVE;

public class BulkNeatController extends Controller<MOVE>{

	private Activator activator;

	public BulkNeatController(Activator activator){
		this.activator = activator;
	}
	
	public MOVE getMove(Game game, long timeDue){
		
		DataTuple dt = new DataTuple(game, MOVE.NEUTRAL);
		
		return convertOutput(activator.next(dt.nnInputs()));		
	}
	
	private MOVE convertOutput(double[] outputs) {
		
		int bestIndex = -1;
		double bestValue = Double.NEGATIVE_INFINITY;
		
		for (int i = 0; i < outputs.length; i++) {
			if (outputs[i] > bestValue) {
//				bestValue = outputs[i];
				bestValue = clamp(outputs[i],0,1);
				bestIndex = i;
			}
		}
		
		if (bestIndex == 0) { return MOVE.UP; }
		if (bestIndex == 1) { return MOVE.RIGHT; }
		if (bestIndex == 2) { return MOVE.DOWN; }
		return MOVE.LEFT; // i == 3
	
	}
	
	/**
	 * norm val
	 * @param value
	 * @param min
	 * @param max
	 * @return
	 */
	private double clamp(double value, double min, double max){
		return Math.max(min, Math.min(max, value));
	}
	
}