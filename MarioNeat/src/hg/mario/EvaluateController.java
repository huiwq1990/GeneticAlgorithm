package hg.pacman;

import static pacman.game.Constants.DELAY;

import org.jgap.Chromosome;

import pacman.controllers.Controller;
import pacman.game.Game;
import pacman.game.Constants.MOVE;

import com.anji.integration.Activator;

public class EvaluateController {
	
	private int numTrials;

	public void evaluate( Chromosome c ) {
		try {
			Activator activator = factory.newActivator( c );

			// calculate fitness, sum of multiple trials
			int fitness = 0;
			for ( int i = 0; i < numTrials; i++ )
				fitness += singleTrial( activator );
			c.setFitnessValue( fitness );
		}
		catch ( Throwable e ) {
//			logger.warn( "error evaluating chromosome " + c.toString(), e );
			c.setFitnessValue( 0 );
		}
	}
	
	
	public void singleTrial(Activator activator){
		 Game  game=new Game(rnd.nextLong());			
					while(!game.gameOver())
					{
				        game.advanceGame( (( Controller<MOVE>)population[which]).getMove(game.copy(),System.currentTimeMillis()+DELAY),
				        		ghostController.getMove(game.copy(),System.currentTimeMillis()+DELAY));
					}			
					fitness[which]+=game.getScore();         
	}

}
