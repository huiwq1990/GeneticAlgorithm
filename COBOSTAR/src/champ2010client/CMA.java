/**
 * 
 */
package champ2010client;

import java.util.StringTokenizer;

import cma.CMAEvolutionStrategy;

class ControllerFitness
{
	private static int UDP_TIMEOUT = 100;
	private static int port = 3001;
	private static String host = "localhost";
	private static String clientId = "championship2009";
	private static boolean verbose;
	private static int maxSteps;
	private SocketHandler mySocket;
	private final int maxDamage = 1000;
	
	private CMAEvolvable driver;
	
	public ControllerFitness (String[] args)
	{
		driver = (CMAEvolvable) load(args[0]);
		parseParameters(args);
	}
	
	public double valueOf (double[] x) {
		driver.reset();
		driver.setParameters(x);
		double fitness = 1 / Math.max(1, drive()+1);
		System.out.println("Fitness: " + fitness);
		return fitness;
	}
	
	public double[] getParameters()
	{
		return driver.getParameters();
	}
	
	public double[] getInitialDeviationParameters()
	{
		return driver.getInitialDeviationParameters();
	}
	
	public boolean isFeasible(double[] x) {
		driver.setParameters(x);
		return driver.isFeasible();
	}
	
	public void shutdown()
	{
		mySocket.close();
	}

	private double drive()
	{
		String inMsg;
		mySocket = new SocketHandler(host, port, verbose);

		/*
		 * Client identification
		 */

		do {
			mySocket.send(clientId);
			inMsg = mySocket.receive(UDP_TIMEOUT);
		} while (inMsg == null || inMsg.indexOf("***identified***") >= 0);

		/*
		 * Start to drive
		 */
		long currStep = 0;
		while (true) {
			/*
			 * Receives from TORCS the game state
			 */
			inMsg = mySocket.receive(UDP_TIMEOUT);

			if (inMsg != null) {

				/*
				 * Check if race is ended (shutdown)
				 */
				if (inMsg.indexOf("***shutdown***") >= 0) {
					System.out.println("Server shutdown!");
					break;
				}

				/*
				 * Check if race is restarted
				 */
				if (inMsg.indexOf("***restart***") >= 0) {
					driver.reset();
					if (verbose)
						System.out.println("Server restarting!");
					break;
				}

				Action action = new Action();
				if (currStep < maxSteps || maxSteps == 0)
					action = driver.control(new MessageBasedSensorModel(
							inMsg));
				else
				{
					action.restartRace = true;
				}
				/*
				if((new MessageBasedSensorModel(inMsg)).getDamage() > maxDamage) {
					System.out.println("Reached max damage: "+maxDamage);
					action.restartRace = true;
				}
				*/
				if(currStep > maxSteps-1) {//ML
					System.out.println("Time is up!");
					action.restartRace = true;
				}
				
				currStep++;
				mySocket.send(action.toString());
				
				if (action.restartRace == true)
				{
					mySocket.close();
					System.out.println("The evolved driver dealt "+(new MessageBasedSensorModel(inMsg)).getOtherdamage()+" points of damage to the other car.");
					System.out.println("The evolved driver received "+(new MessageBasedSensorModel(inMsg)).getDamage()+" points of damage by the other car.");
					return (new MessageBasedSensorModel(inMsg)).getOtherdamage() / 10000;//Change this fitness function according to your needs!
//					Use getOtherdamage() for measuring the damage that your evolvable client has dealt to the training client,
//					use getDamage() for measuring the damage that your evolvable client received by the training client.
//					IMPORTANT: getOtherDamage() is meant for usage with one training client only. 
//					Moreover, it is only meant for evolution purposes prior to the competition. It will NOT be supported in the actual competition!!!
				}
			} else
				System.out.println("Server did not respond within the timeout");
		}

		mySocket.close();
		return 0;
	}
	
	private static void parseParameters(String[] args) {
		/*
		 * Set default values for the options
		 */
//		verbose = false;
//		maxSteps = 0;
		verbose = false;
		maxSteps = 10000;

		for (int i = 1; i < args.length; i++) {
			StringTokenizer st = new StringTokenizer(args[i], ":");
			String entity = st.nextToken();
			String value = st.nextToken();
			if (entity.equals("verbose")) {
				if (value.equals("on"))
					verbose = true;
				else if (value.equals(false))
					verbose = false;
				else {
					System.out.println(entity + ":" + value
							+ " is not a valid option");
					System.exit(0);
				}
			}
			if (entity.equals("id")) {
				clientId = value;

			}
			if (entity.equals("maxSteps")) {
				maxSteps = Integer.parseInt(value);
				if (maxSteps < 0) {
					System.out.println(entity + ":" + value
							+ " is not a valid option");
					System.exit(0);
				}

			}

		}
	}

	private static Controller load(String name) {
		Controller controller=null;
		try {
			controller = (Controller) (Object) Class.forName(name)
					.newInstance();
		} catch (ClassNotFoundException e) {
			System.out
					.println(name
							+ " is not a class name");
			System.exit(0);
		} catch (InstantiationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return controller;
	}
}

/**
 * @author Daniele Loiacono
 * 
 */
public class CMA {

	/**
	 * @param args
	 *            is used to define all the options of the client.
	 *            <port:N> is used to specify the port for the connection (default is 3001)
	 *            <host:ADDRESS> is used to specify the address of the host where the server is running (default is localhost)  
	 *            <id:ClientID> is used to specify the ID of the client sent to the server (default is championship2009) 
	 *            <verbose:on> is used to set verbose mode on (default is off)
	 *            <maxEpisodes:N> is used to set the number of episodes (default is 1)
	 *            <maxSteps:N> is used to set the max number of steps for each episode (0 is default value, that means unlimited number of steps)
	 *            - 
	 */
	public static void main(String[] args)
	{
		String[] ss={"champ2010client.COBOSTAR"};
		ControllerFitness fitfun = new ControllerFitness(ss);
		
		// new a CMA-ES and set some initial values
		CMAEvolutionStrategy cma = new CMAEvolutionStrategy();
		cma.readProperties(); // read options, see file CMAEvolutionStrategy.properties
		cma.setDimension(fitfun.getParameters().length); // overwrite some loaded properties
		cma.setInitialX(fitfun.getParameters()); // in each dimension, also setTypicalX can be used
		cma.setInitialStandardDeviations(fitfun.getInitialDeviationParameters()); // also a mandatory setting 
		cma.options.stopFitness = 1e-9;       // optional setting

		// initialize cma and get fitness array to fill in later
		double[] fitness = cma.init();  // new double[cma.parameters.getPopulationSize()];

		// initial output to files
		cma.writeToDefaultFilesHeaders(0); // 0 == overwrites old files

		// iteration loop
		while(cma.stopConditions.getNumber() == 0) {

            // --- core iteration step ---
			double[][] pop = cma.samplePopulation(); // get a new population of solutions
			for(int i = 0; i < pop.length; ++i) {    // for each candidate solution i
            	// a simple way to handle constraints that define a convex feasible domain  
            	// (like box constraints, i.e. variable boundaries) via "blind re-sampling" 
            	                                       // assumes that the feasible domain is convex, the optimum is  
				while (!fitfun.isFeasible(pop[i]))     //   not located on (or very close to) the domain boundary,  
					pop[i] = cma.resampleSingle(i);    //   initialX is feasible and initialStandardDeviations are  
                                                       //   sufficiently small to prevent quasi-infinite looping here
                // compute fitness/objective value	
				fitness[i] = fitfun.valueOf(pop[i]); // fitfun.valueOf() is to be minimized
			}
			cma.updateDistribution(fitness);         // pass fitness array to update search distribution
            // --- end core iteration step ---

			// output to files and console 
			cma.writeToDefaultFiles();
			int outmod = 150;
			if (cma.getCountIter() % (15*outmod) == 1)
				cma.printlnAnnotation(); // might write file as well
			if (cma.getCountIter() % outmod == 1)
				cma.println(); 
		}
		// evaluate mean value as it is the best estimator for the optimum
		cma.setFitnessOfMeanX(fitfun.valueOf(cma.getMeanX())); // updates the best ever solution 

		// final output
		cma.writeToDefaultFiles(1);
		cma.println();
		cma.println("Terminated due to");
		for (String s : cma.stopConditions.getMessages())
			cma.println("  " + s);
		cma.println("best function value " + cma.getBestFunctionValue() 
				+ " at evaluation " + cma.getBestEvaluationNumber());
			
		// we might return cma.getBestSolution() or cma.getBestX()
	}
}
