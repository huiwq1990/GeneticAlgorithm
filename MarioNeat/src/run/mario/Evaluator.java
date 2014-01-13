package run.mario;

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

import hg.mario.MarioFitnessFunction;

public class Evaluator {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub

		String[] parms = {"mario.properties","7253"};		
		args = parms;
		
		MarioFitnessFunction ff = new MarioFitnessFunction();
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
