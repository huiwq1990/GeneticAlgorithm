package itu.jgdiejuu.torcs;

import java.util.ArrayList;
import java.util.List;

import org.jgap.Chromosome;

import com.anji.integration.ActivatorTranscriber;
import com.anji.integration.TranscriberException;

public class BulkNEATController extends Controller {
	
	private static final int MAX_STEPS = 8000; // about 2½ minute
	
	private boolean finished = false;
	private ArrayList<Integer> fitnesses;
	
	private List<Chromosome> genotypes;
	private NEATController controller = null;
	private int curStep = 0;
	private int curGene = 0;

	private ActivatorTranscriber factory;
	private boolean manualGear;

	public BulkNEATController(List<Chromosome> genotypes, ActivatorTranscriber activatorFactory, boolean manualGear){
		this.manualGear = manualGear;
		this.genotypes = genotypes;
		this.factory = activatorFactory;
		fitnesses = new ArrayList<Integer>(genotypes.size());
		
		createController();
	}

	@Override
	public Action control(SensorModel sensors) {
		if(curStep < MAX_STEPS && !disqualified()){
			
			curStep++;
			return controller.control(sensors);
		}else{
			Action result = new Action();
			
			// print reason
			if(disqualified()){
				System.out.println(">>\tController Stuck: d="+controller.getDeltaFive()+" step "+curStep);
			}else{
				System.out.println(">>\tController Out of Time: "+curStep+" = "+MAX_STEPS);
			}
			System.out.println(">>\t"+controller.toString());
			
			// save fitness
			fitnesses.add(controller.getMaxDistance());
			
			// change variables
			curGene++;
			curStep = 0;
			
			// go to next
			if(curGene < genotypes.size()){
				createController();
				result.restartRace = true;
			}else{
				//System.out.println("BULK out of genes");
				finished = true;
			}
			
			return result;
		}
	}
	
	private boolean disqualified() {
		return !(controller.getDeltaFive() >= 1.0 /*|| controller.getDeltaFive() < 0*/);
	}

	public boolean isFinished(){
		return finished;
	}
	
	public List<Integer> getFitnesses(){
		return fitnesses;
	}
	
	private void createController() {
		try {
			System.out.println(">> Controller #"+(curGene+1) + " - id: "+genotypes.get(curGene).getId());
			controller = new NEATController(factory.newActivator(genotypes.get(curGene)),manualGear);
		} catch (TranscriberException e) { e.printStackTrace(); controller = null; }
		
	}

	@Override
	public void reset() {
		//System.out.println(">> Reset call received");
	}

	@Override
	public void shutdown() { }

}
