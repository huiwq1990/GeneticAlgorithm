/**
 * 
 */

package itu.jgdiejuu.torcs;

import itu.jgdiejuu.torcs.Controller.Stage;

import java.util.StringTokenizer;


/**
 * @author Daniele Loiacono
 * 
 */
public class ClientProgramA {

	private static int UDP_TIMEOUT = 10000;
	private static int port;
	private static String host;
	private static String clientId;
	private static boolean verbose;
	private static int maxEpisodes;
	private static int maxSteps;
	private static Stage stage;
	private static String trackName;

	/**
	 * @param args
	 *            is used to define all the options of the client.
	 *            <port:N> is used to specify the port for the connection (default is 3001)
	 *            <host:ADDRESS> is used to specify the address of the host where the server is running (default is localhost)  
	 *            <id:ClientID> is used to specify the ID of the client sent to the server (default is championship2009) 
	 *            <verbose:on> is used to set verbose mode on (default is off)
	 *            <maxEpisodes:N> is used to set the number of episodes (default is 1)
	 *            <maxSteps:N> is used to set the max number of steps for each episode (0 is default value, that means unlimited number of steps)
	 *            <stage:N> is used to set the current stage: 0 is WARMUP, 1 is QUALIFYING, 2 is RACE, others value means UNKNOWN (default is UNKNOWN)
	 *            <trackName:name> is used to set the name of current track
	 */
	public static void main(String[] args, Controller driver) {
		parseParameters(args);
		SocketHandler mySocket = new SocketHandler(host, port, verbose);
		String inMsg;

		driver.setStage(stage);
		driver.setTrackName(trackName);
		
		/* Build init string */
		float[] angles = driver.initAngles();
		String initStr = clientId + "(init";
		for (int i = 0; i < angles.length; i++) {
			initStr = initStr + " " + angles[i];
		}
		initStr = initStr + ")";
		
		long curEpisode = 0;
		boolean shutdownOccurred = false;
		System.out.println("Client Connected.");
		do {

			/*
			 * Client identification
			 */

			do {
				mySocket.send(initStr);
				inMsg = mySocket.receive(UDP_TIMEOUT);
			} while (inMsg == null || inMsg.indexOf("***identified***") < 0);
			
			//System.out.println("New Episode");
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
						shutdownOccurred = true;
						System.out.println("Server shutdown!");
						break;
					}
					
					/*
					 * Check if BulkNEATController is done
					 */
					if (driver instanceof BulkNEATController && ((BulkNEATController)driver).isFinished()) {
						shutdownOccurred = true;
						//System.out.println("BulkNEATController done!");
						break;
					}

					/*
					 * Check if race is restarted
					 */
					if (inMsg.indexOf("***restart***") >= 0) {
						driver.reset();
						//System.out.println("Server resetting race!");
						break;
					}

					Action action = new Action();
					if (currStep < maxSteps || maxSteps == 0)
						action = driver.control(new MessageBasedSensorModel(
								inMsg));
					else
						action.restartRace = true;

					currStep++;
					mySocket.send(action.toString());
				} else
					System.out.println("Server did not respond within the timeout");
			}

		} while (++curEpisode < maxEpisodes && !shutdownOccurred);

		/*
		 * Shutdown the controller
		 */
		driver.shutdown();
		mySocket.close();
		if (verbose)System.out.println("Client shutdown.");
		if (verbose)System.out.println("Bye, bye!");

	}

	private static void parseParameters(String[] args) {
		/*
		 * Set default values for the options
		 */
		port = 3001;
		host = "localhost";
		clientId = "championship2010";
		verbose = false;
		maxEpisodes = 1;
		maxSteps = 0;
		stage = Stage.UNKNOWN;
		trackName = "unknown";
		
		for (int i = 1; i < args.length; i++) {
			StringTokenizer st = new StringTokenizer(args[i], ":");
			String entity = st.nextToken();
			String value = st.nextToken();
			if (entity.equals("port")) {
				port = Integer.parseInt(value);
			}
			if (entity.equals("host")) {
				host = value;
			}
			if (entity.equals("id")) {
				clientId = value;
			}
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
			if (entity.equals("stage")) {
				stage = Stage.fromInt(Integer.parseInt(value));
			}
			if (entity.equals("trackName")) {
				trackName = value;
			}
			if (entity.equals("maxEpisodes")) {
				maxEpisodes = Integer.parseInt(value);
				if (maxEpisodes <= 0) {
					System.out.println(entity + ":" + value
							+ " is not a valid option");
					System.exit(0);
				}
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
		
		if (verbose) System.out.println("port="+port);
		if (verbose) System.out.println("host="+host);
		if (verbose) System.out.println("clientId="+clientId);
		if (verbose) System.out.println("verbose="+verbose);
		if (verbose) System.out.println("maxEpisodes="+maxEpisodes);
		if (verbose) System.out.println("maxSteps="+maxSteps);
		if (verbose) System.out.println("stage="+stage);
		if (verbose) System.out.println("trackName="+trackName);
	}
}
