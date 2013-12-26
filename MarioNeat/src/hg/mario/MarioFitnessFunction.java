package hg.mario;


import java.util.Collections;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executor;

import org.jgap.BulkFitnessFunction;
import org.jgap.Chromosome;








import ch.idsia.agents.Agent;
import ch.idsia.agents.controllers.ForwardAgent;
import ch.idsia.benchmark.mario.engine.GlobalOptions;
import ch.idsia.benchmark.tasks.BasicTask;
import ch.idsia.benchmark.tasks.MarioCustomSystemOfValues;
import ch.idsia.tools.MarioAIOptions;

import com.anji.integration.Activator;
import com.anji.integration.ActivatorTranscriber;
import com.anji.integration.TranscriberException;
import com.anji.util.Configurable;
import com.anji.util.Properties;

@SuppressWarnings("serial")
public class MarioFitnessFunction implements BulkFitnessFunction, Configurable {

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
//			System.out.println(c.getFitnessValue());
		}
		
		
		
	}
	
	/**
	 * Evaluate chromosome and set fitness.
	 * @param c
	 */
	public void evaluate( Chromosome c ) {
//		System.out.println("evaluate");
		try {
			Activator activator = activatorFactory.newActivator(c);

			Agent agent = new BulkNeatController(activator);
//			 Agent agent = new ForwardAgent();
			String[] args = {};
			MarioAIOptions marioAIOptions = new MarioAIOptions(args);
			
			marioAIOptions.setVisualization(false);
//			marioAIOptions.set
			marioAIOptions.setAgent(agent);
			marioAIOptions.setFPS(GlobalOptions.MaxFPS);
			BasicTask basicTask = new BasicTask(marioAIOptions);
			final MarioCustomSystemOfValues m = new MarioCustomSystemOfValues();
			basicTask.setOptionsAndReset(marioAIOptions);
//			basicTask.doEpisodes(1, false, 1);
			basicTask.runSingleEpisode(1);
			// System.out.println("\nEvaluationInfo: \n" +
			// basicTask.getEnvironment().getEvaluationInfoAsString());
			
			double fitness = basicTask.getEnvironment().getEvaluationInfo()
					.computeWeightedFitness(m);
			
//			System.out.println("fitness:" + fitness);
			c.setFitnessValue((int) fitness);
		}
		catch ( Throwable e ) {
//			logger.warn( "error evaluating chromosome " + c.toString(), e );
			c.setFitnessValue( 0 );
			System.out.println(e);
		}
	}
	
	
	
//	public void evaluate( Chromosome c ) {
//		System.out.println("evaluate");
//	
//			Activator activator = null;
//			try {
//				activator = activatorFactory.newActivator(c);
//			} catch (TranscriberException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//
//			Agent agent = new BulkNeatController(activator);
////			 Agent agent = new ForwardAgent();
//			String[] args = {};
//			MarioAIOptions marioAIOptions = new MarioAIOptions(args);
//			BasicTask basicTask = new BasicTask(marioAIOptions);
//			marioAIOptions.setVisualization(false);
//			marioAIOptions.setAgent(agent);
//
//			final MarioCustomSystemOfValues m = new MarioCustomSystemOfValues();
//
//			basicTask.doEpisodes(1, false, 1);
//			// System.out.println("\nEvaluationInfo: \n" +
//			// basicTask.getEnvironment().getEvaluationInfoAsString());
//			
//			double fitness = basicTask.getEnvironment().getEvaluationInfo()
//					.computeWeightedFitness(m);
//			
//			System.out.println("fitness:" + fitness);
//			c.setFitnessValue((int) fitness);
//		
//	}
	
	
	
	public void evaluateBest( Chromosome c ) {
		try {
			Activator activator = activatorFactory.newActivator( c );
			
			final Agent agent = new BulkNeatController(activator);

			final MarioAIOptions marioAIOptions = new MarioAIOptions();
			final BasicTask basicTask = new BasicTask(marioAIOptions);
			marioAIOptions.setVisualization(true);
			marioAIOptions.setAgent(agent);

			final MarioCustomSystemOfValues m = new MarioCustomSystemOfValues();

			basicTask.doEpisodes(1, false, 1);
			// System.out.println("\nEvaluationInfo: \n" +
			// basicTask.getEnvironment().getEvaluationInfoAsString());
			// System.out.println("\nCustom : \n" +
			// basicTask.getEnvironment().getEvaluationInfo().computeWeightedFitness(m));
			
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
