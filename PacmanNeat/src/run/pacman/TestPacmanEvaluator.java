package run.pacman;

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

public class TestPacmanEvaluator {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub

		String[] parms = {"pacman.properties","42819"};		
		args = parms;
//		
//		Properties props = new Properties();
//		props.loadFromResource( args[ 0 ] );
//		BulkFitnessFunction fitnessFunc = (BulkFitnessFunction) props
//				.singletonObjectProperty( Evolver.FITNESS_FUNCTION_CLASS_KEY );
//
//		// load chromosomes
//		Persistence db = (Persistence) props.newObjectProperty( Persistence.PERSISTENCE_CLASS_KEY );
//		Configuration config = new DummyConfiguration();
//		ArrayList chroms = new ArrayList();
//		for ( int i = 1; i < args.length; ++i ) {
//			Chromosome chrom = db.loadChromosome( args[ i ], config );
//			if ( chrom == null )
//				throw new IllegalArgumentException( "no chromosome found: " + args[ i ] );
//			chroms.add( chrom );
//		}
//
//		// evaluate
//		fitnessFunc.evaluate( chroms );
//
//		Iterator it = chroms.iterator();
//		while ( it.hasNext() ) {
//			Chromosome chrom = (Chromosome) it.next();
////			logger.info( chrom.toString() + ": fitness = " + chrom.getFitnessValue() + "/"
////					+ fitnessFunc.getMaxFitnessValue() );
//		}
		
		
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
