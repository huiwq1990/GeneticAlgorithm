package hg.torcs;

import static pacman.game.Constants.DELAY;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executor;

import org.jgap.BulkFitnessFunction;
import org.jgap.Chromosome;

import pacman.controllers.Controller;
import pacman.controllers.examples.StarterGhosts;
import pacman.controllers.examples.StarterPacMan;
import pacman.game.Game;
import pacman.game.Constants.GHOST;
import pacman.game.Constants.MOVE;

import com.anji.integration.Activator;
import com.anji.integration.ActivatorTranscriber;
import com.anji.util.Configurable;
import com.anji.util.Properties;

@SuppressWarnings("serial")
public class PacmanFitnessFunction implements BulkFitnessFunction, Configurable {

	//private static final int MAX_STEPS = 1500;
	
	private static final int MAX_FITNESS = 13337;
	private ActivatorTranscriber activatorFactory;
	
	public void init( Properties props ){
		activatorFactory = (ActivatorTranscriber) props
				.singletonObjectProperty( ActivatorTranscriber.class );
	}
		
	public int getMaxFitnessValue() {
		return MAX_FITNESS;
	}

	@Override
	final public void evaluate(List<Chromosome> genotypes) {
		// evaluate each chromosome
		Iterator<Chromosome> it = genotypes.iterator();
		while ( it.hasNext() ) {
			Chromosome c = (Chromosome) it.next();
			evaluate( c );
		}
		
		
		
	}
	
	/**
	 * Evaluate chromosome and set fitness.
	 * @param c
	 */
	public void evaluate( Chromosome c ) {
		try {
			Activator activator = activatorFactory.newActivator( c );
			// calculate fitness, sum of multiple trials
			Controller<MOVE> pacManController = new BulkNeatController(activator);
			Controller<EnumMap<GHOST,MOVE>> ghostController = new StarterGhosts();			
			
			double fitness = PacmanEvaluate.runExperiment(pacManController, ghostController, 1);
			
			c.setFitnessValue((int) fitness);
		}
		catch ( Throwable e ) {
//			logger.warn( "error evaluating chromosome " + c.toString(), e );
			c.setFitnessValue( 0 );
		}
	}
	
	
	
	public void evaluateBest( Chromosome c ) {
		try {
			Activator activator = activatorFactory.newActivator( c );
			// calculate fitness, sum of multiple trials
			Controller<MOVE> pacManController = new BulkNeatController(activator);
			Controller<EnumMap<GHOST,MOVE>> ghostController = new StarterGhosts();			
			
			PacmanEvaluate.runGameTimed(pacManController, ghostController, true);
			
//			c.setFitnessValue((int) fitness);
		}
		catch ( Throwable e ) {
//			logger.warn( "error evaluating chromosome " + c.toString(), e );
			c.setFitnessValue( 0 );
		}
	}
	
//	public double singleTrial(Activator activator){
//	
//		Controller<MOVE> pacManController = new BulkNeatController(activator);
//		Controller<EnumMap<GHOST,MOVE>> ghostController = new StarterGhosts();
//		
//		
//		double result = PacmanEvaluate.runExperiment(pacManController, ghostController, 1);
//		return result;
////		while (!game.gameOver()) {
////			game.advanceGame(pacmanControl.getMove(game.copy(),	System.currentTimeMillis() + DELAY),ghostController.getMove(game.copy(),System.currentTimeMillis() + DELAY));
////		}		
////		
////		return game.p
//	}

}
