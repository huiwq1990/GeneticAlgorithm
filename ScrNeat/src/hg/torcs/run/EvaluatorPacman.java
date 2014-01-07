package hg.torcs.run;

import java.util.ArrayList;
import java.util.Iterator;

import org.jgap.BulkFitnessFunction;
import org.jgap.Chromosome;
import org.jgap.Configuration;

import com.anji.neat.Evolver;
import com.anji.persistence.Persistence;
import com.anji.polebalance.DoublePoleBalanceFitnessFunction;
import com.anji.util.DummyConfiguration;
import com.anji.util.Properties;

import hg.pacman.PacmanFitnessFunction;
import itu.jgdiejuu.torcs.Evaluator;

public class EvaluatorPacman {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub

		String[] parms = {"pacman.properties","65363"};		
		args = parms;
		
		PacmanFitnessFunction ff = new PacmanFitnessFunction();
		Properties props = new Properties();
		props.loadFromResource( args[ 0 ] );
		ff.init( props );
		Persistence db = (Persistence) props.newObjectProperty( Persistence.PERSISTENCE_CLASS_KEY );
		Configuration config = new DummyConfiguration();
		Chromosome chrom = db.loadChromosome( args[ 1 ], config );
		if ( chrom == null )
			throw new IllegalArgumentException( "no chromosome found: " + args[ 1 ] );
//		ff.enableDisplay();
		ff.evaluateBest( chrom );
//		logger.info( "Fitness = " + chrom.getFitnessValue() );
	}

}
