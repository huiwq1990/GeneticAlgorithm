package champ2010client;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import champ2010client.LeastSquaresInterpolation.Result;

/**
 * COBOSTAR
 * 
 * COgnitive BOdySpaces: TORCS Autonomous Racer 
 * 
 * Version: March/2010
 * 
 * Thies Lönneker
 * Matthias Linhardt
 * Martin Butz, PhD
 * Julius-Maximilians-Universität Würzburg
 */

public class COBOSTAR extends CMAEvolvable implements ChangeListener, ActionListener
{
//	int counter = 1000;
//	int negative =0;
	private enum ParameterSet
	{
		ONTRACK, OFFTRACK, ASR, ASROFFROAD, STEERING
	}

	private final boolean doDemolitionDerby = false;
	// if this is a DEMOLITION DERBY, i.e. we want to crash other cars by purpose
	private boolean doSimpleAvoidance = true;
	private double simpleAvoidanceDistance = 15; //35;
	private double sophisticatedAvoidanceDistance = 60;
	// if opponent avoidance should apply
	private boolean avoidOpponents = true;
	// If the track monitor should be shown
	private final boolean doShowTrackMonitor = false;
	private final boolean doShowOutputs = false;
	private final boolean useFilter = false;
	private final boolean useInterpolation = false;
	// shall we view the control values and enable manipulating them online?
	private final boolean doValueControl = false;
	// Shall we try to switch strategies after the first round? 
	private boolean doSwitchDrivingStyle = true;
	// Shall we adjust the steering values on wide tracks?
	private final boolean doAdjustSpeedwaySteering = true;
	// Shall we remember crash positions and slow down if we approach them again?
	private boolean doCrashLocationPrevention = true;
	// Which parameter set is going to be evolved?
	private final ParameterSet evolveParameterSet = ParameterSet.ONTRACK;
	// Shall deliberate crashes happen?
	private final boolean doDeliberateCrashing = false;
	// Shall we try to detect Offroad Tracks?
	private final boolean doDetectOffroadTrack = true;
	// Shall we prepare for safe landing if we think we jumped over a crest?
	private boolean doJumpDetection = true;
	// Should we be careful with the gas pedal when off-track?
	private boolean doSpeedReduction = true;
	private final boolean doSpeedReductionOnTrack = true;
	// Shall we switch to reverse if scraping along a wall?
	private boolean doStuck2Detection = true;

	public static final boolean doSysouts = false;//activate text output

	/* Gear changing constants */
	private final int[]  gearUp = {9500, 9400, 9500, 9500, 9500, 0};
	private final int[]  gearDown =  {0, 3300, 6200, 7000, 7300, 7700};

	/* ABS filter constants */
	private final float wheelRadius[] = {0.3179f, 0.3179f, 0.3276f, 0.3276f};
	private final float absMinSpeed = 11f;

	public static double trackLength = 0;
	
	/* Steering constant */
	private double angularAdjustments = 0.500;

	/* Crash generator constants */
	private final int crashDistance = 300;
	private final int crashDuration = 50;
	private int crashCounter;
	private int crashActiveCounter;

	private enum Behavior
	{
		NORMAL, REVERSE, BRAKE
	}

	private enum DrivingStyle
	{
		SLOWEST, SLOW, NORMAL, FAST, DIRTTRACK
	}

	private Behavior behavior;
	private int stuckCounter;
	private double stuckAngle;
	private int waitForGearSwitch;
	private int gear;

	private int focusAngle;//Angle of the focus sensors (track edge sensors with better resolution) 
	//Variables for Demolition Derby:
	int reverseCounter = 0;//counts time of backwards driving
	int normalCounter = 0;//counts time of regular behavior in demolition derby
	int turnOver = 0;//!=0 indicates u-turning
	boolean stalled = false;//true if the opponent is pushing you backwards

	private boolean offRoadTrack;

	private double[] initialDeviationParameters;
	private final double[] initialOntrackDeviationParameters = {
			10.0, 0.5, 30.0, 20.0, 1.5, 100.0, 1.0, 5.0, 10.0, .4, .3, 2.0, 2.0, .15, .4, .3};
	private final double[] initialOfftrackDeviationParameters = {
			/*.1,*/ 10.0, .1, .2, 20.0, 4.0, 1.5, 10.0};
	private final double[] initialASRDeviationParameters = {
			3.0, 2.0};
	private final double[] initialSteeringDeviationParameters = {
			20.0};

	private Map<String, Double> onTrackValues = new LinkedHashMap<String, Double>();

	private Map<String, Double> steeringValues = new LinkedHashMap<String, Double>();
	private Map<String, Double> offTrackValues = new LinkedHashMap<String, Double>();
	private Map<String, Double> ASRValues = new LinkedHashMap<String, Double>();
	private Map<String, Double> offRoadASRValues = new LinkedHashMap<String, Double>();

	private Map<String, Double> evolvedValues;

	private Vector<Car> opponentData;

	private boolean firstRoundCompleted;
	private double speedMin;
	private double speedMax;

	private double trackWidth;

	private MonitorPanel monitor;

	private CrashLocationPreventer crashLocationPreventer;

	private int wasOffTrackCounter = 0;

	private final int wasOffTrackIntegration = 30;

	// Variables for Jump Detection (don't make abrupt actions before you have safely landed):
	private int jumpDetection = 0;
	private boolean jump = false;
	private double jumpTime = 0;
	private double jumpSpeed = 0;

	// Off-track Speed Reduction (Be careful with the gas pedal when off track):
	private double speedReduction = 0; 
	private boolean offTrackSpeedReduction = false;

	// Stuck2: grinding along a wall with the car's outer front edge:
	private int stuck2Counter;//second stuck counter for scraping along a wall with the car's front

	private final double tickTime = .02;
	private final float carLength = 4.5F;

	// variables for detecting a rear wheel being off-track:
	private double x = 0; //minimum distance that the car has to have from the track edge in order to be on-track with its rear
	private double b = 1.5; //width of the car
	private double d = Math.sqrt(Math.pow(0.5*b, 2) + Math.pow(0.5*carLength, 2)); //distance of car's center and its outer rear edge 

	private SomeOutputWindow outputWindow;
	private KalmanFilter edgeDistanceFilter = new KalmanFilter(outputWindow);
	
	public COBOSTAR ()
	{
		if(doShowOutputs) outputWindow = new SomeOutputWindow();
		double help = 0;
		for (int i = 0; i<filterTime; i++)
		{
			help += Math.pow(decay, i);
		}
		norm = help;
		
		
		//Switch off everything that lessens destructive power when in demolition derby mode:
		if(doDemolitionDerby){
			avoidOpponents = false;
			doSimpleAvoidance = false;
			doCrashLocationPrevention = false;
			if(doSysouts)
				System.out.println("Destruction Derby Mode: Go crash them all!");
		}

		// Off-track Values
		//		offTrackValues.put("TargetSpeed", 117.49172620647438);
		offTrackValues.put("TargetSpeed", 90.);
		offTrackValues.put("InnerTrackBorder", 0.39242375307675126);
		offTrackValues.put("TrackBorderDirectionFactor", 0.1500973884529463);
		offTrackValues.put("TargetSpeedFactor", 123.55954065981577);
		offTrackValues.put("DirectionSpeedInfluence", 34.56107277127619);
		offTrackValues.put("StuckSpeed", 2.0265626106014047);
		offTrackValues.put("StuckCounterMax", 53.327755915319855);

		// Values from E34
		ASRValues.put("ASRSlip", 3.2);
		ASRValues.put("ASRRange", 10.86);

		// Values from E35 c
		offRoadASRValues.put("ASRSlip", -10.013742994175);
		offRoadASRValues.put("ASRRange", 154.950734939263);

		switch (evolveParameterSet)
		{
		case ONTRACK:
			initialDeviationParameters = initialOntrackDeviationParameters;
			evolvedValues = onTrackValues;
			break;
		case OFFTRACK:
			initialDeviationParameters = initialOfftrackDeviationParameters;
			evolvedValues = offTrackValues;
			break;
		case ASR:
			initialDeviationParameters = initialASRDeviationParameters;
			evolvedValues = ASRValues;
			break;
		case ASROFFROAD:
			initialDeviationParameters = initialASRDeviationParameters;
			evolvedValues = offRoadASRValues;
			break;
		case STEERING:
			initialDeviationParameters = initialSteeringDeviationParameters;
			evolvedValues = steeringValues;
			break;
		}

		opponentData = new Vector<Car>();

		this.reset();

		/* activating visualization */
		if(doValueControl)
			activateValueControl();

		if(doShowTrackMonitor)
			activateMonitor();

		crashLocationPreventer = new CrashLocationPreventer(this);
	}
	
	private static final int distanceSteps = 10;
	private static final int min = 0; //min and max for the index of the distance grid
	private static final double stepSize = 200/distanceSteps;
	private static final int filterTime = 10;//interpolation length
	private LinkedList<double[]> prevSensors = new LinkedList<double[]>();//unweighted, only used to retrieve oldest value
	private double[][] prevSensorsAverage = new double[36][distanceSteps];//sum over time
	private static final double decay = 0.9;//decay of weighted sensor information over time
	private static final double lastTimeStepDecay = Math.pow(decay, filterTime);//intermediate result to speed up computation time
	private final double norm;
	public static final int maxDebugTime=2;
	private int stepCounter=-1;
	private int startTime=-1;
	private static final double maxUsedRange = 170;
	
	//Calculates the indices in a 2D-grid for the specified distances:
	public static int calculateGridEntries(double[] distances, int[][]indices)
	{
		int count=0;
		//int[][] ret = new int[36][distanceSteps];
		int numberSensors = distances.length;
		for(int i=0; i<numberSensors; i++)
		{
			if(distances[i]<maxUsedRange) {
				count++;
				int index = (int)(distances[i] / stepSize);
				//index = in(index, min, distanceSteps-1);
				if(index<0) index=0;
				indices[i][index]++;
			}
		}
		return count;
		//return ret;
	}
	
	/*public static int in(int value, int min, int max)
	{
		if (value<min)
			return min;
		return value > max ? max : value;
	}*/
	
	/*public static double[] getMaxima(double[][] indices, int number) {
		double[] ret = new double[36];
		TreeMap<Double, int[]> map = new TreeMap<Double, int[]>(); //value[0]: angle, value[1] distance
		for(int i=0; i<indices.length; i++)
			for(int j=0; j<indices[0].length; j++)
				map.put(new Double(indices[i][j]), new int[]{i,j});
		for(int i=0; i<number; i++) {
			Map.Entry<Double, int[]> max = map.lastEntry();
			map.remove(max);
			ret[max.getValue()[0]] = (max.getValue()[1]+0.5)*stepSize;
		}
		return ret;
	}*/
	
	/**
	 * searches for the (angle,distance)-points with highest activation(i.e. times of occurrence in measurements)
	 */
	public static double[] getMaxima(double[][] indices, int number) {
		double[] max = new double[number]; //first element: lowest maximum
		int[] angle = new int[number], distance = new int[number];
		for(int i=0; i<indices.length; i++)
			for(int j=0; j<indices[0].length; j++)
				for(int k=number-1; k>=0; k--) {
					if(indices[i][j]>max[k]) {
						for(int l=0; l<k; l++) {
							max[l]=max[l+1];
							angle[l]=angle[l+1];
							distance[l]=distance[l+1];
						}
						max[k]=indices[i][j];
						angle[k]=i;
						distance[k]=j;
					}
					break;
				}
		double[] ret = new double[36];
		for(int i=0; i<36; i++) ret[i]=200;
		for(int i=0; i<number; i++) ret[angle[i]]=(distance[i]+0.5)*stepSize;
		return ret;
	}
	
	public void out(String caption, double[] values) {
		String s = caption+": ";
		for(int i=0; i<values.length; i++) s+=values[i]+(i<values.length-1 ? ", " : "");
		if(stepCounter<maxDebugTime) System.out.println(s);
	}
	
	public void out(String caption, int[][] values) {
		if(stepCounter<maxDebugTime) System.out.println(caption+": ");
		for(int i=0; i<values.length; i++) {
			String s = "";
			for(int j=0; j<values[i].length; j++) 
				s+=values[i][j]+(j<values[i].length-1 ? ", " : "");
			if(stepCounter<maxDebugTime) System.out.println(s);
		}
	}
	
	public void out(String caption, double[][] values) {
		if(stepCounter<maxDebugTime) System.out.println(caption+": ");
		for(int i=0; i<values.length; i++) {
			String s = "";
			for(int j=0; j<values[i].length; j++) 
				s+=values[i][j]+(j<values[i].length-1 ? ", " : "");
			if(stepCounter<maxDebugTime) System.out.println(s);
		}
	}
	
	public double[] filterOpponents(double[] sensors)//Stephan's and Matthias' best filter device in the world!
	{
		//if(stepCounter<maxDebugTime) System.out.println("stepCounter: "+stepCounter);
		//out("at filterOpponents start: sensors", sensors);
		int size = prevSensors.size();
		int numOpponents;
		if(size >= filterTime)
		{
			//remove the oldest sensor values:
			double[] first = prevSensors.removeFirst();
			int[][] indicesFirst = new int[36][distanceSteps];
			calculateGridEntries(first, indicesFirst);
			multiply(indicesFirst, lastTimeStepDecay);
			minusScaled(prevSensorsAverage, indicesFirst, 1/norm); 
		}
		
		//shifts the sum backwards in time:
		if(size >= 0)
			multiply(prevSensorsAverage, decay);

		//add the newest sensor value:
		prevSensors.addLast(sensors);
		int[][] indicesCurrent = new int[36][distanceSteps];
		numOpponents = calculateGridEntries(sensors, indicesCurrent);
		//if(stepCounter<maxDebugTime) System.out.println("numOpponents");
		//out("adding", indicesCurrent);
		addScaled(prevSensorsAverage, indicesCurrent, 1/norm);
		//out("prevAverage", prevSensorsAverage);
		double[] maxima = getMaxima(prevSensorsAverage, numOpponents);
		//out("maxima", maxima);
		return maxima;
	}
	
	public static void add(double[] first, double[] second)
	{
		for(int i = 0; i < first.length; i++)
		{
			first[i] += second[i];
		}		
	}
	
//	public static void addScaled(double[] first, double[] second, double scale)
//	{
//		for(int i = 0; i < first.length; i++)
//		{
//			first[i] += second[i]*scale;
//		}		
//	}
	
	public static void multiply(int[][] values, double scale)
	{
		for(int i=0; i<values.length; i++)
			for(int j=0; j<values[i].length; j++)
			{
				values[i][j] *= scale;
			}
	}
	
	public static void multiply(double[][] values, double scale)
	{
		for(int i=0; i<values.length; i++)
			for(int j=0; j<values[i].length; j++)
			{
				values[i][j] *= scale;
			}
	}
	
	/*
	public static void minusFirst(double[] first, double[] second)
	{
		for(int i = 0; i < first.length; i++)
		{
			first[i] -= second[i];
		}		
	}*/
	
	public static double[][] minusScaled(double[][] first, int[][] second, double scaled)
	{
		double[][] ret = new double[first.length][first[0].length];
		for(int i = 0; i < ret.length; i++)
			for(int j =0; j<ret[i].length; j++)
			{
				ret[i][j] = first[i][j] - ((double)second[i][j])*scaled;
			}		
		return ret;
	}

	public void addScaled(double[][] first, int[][] second, double scaled)
	{
		for(int i = 0; i < first.length; i++)
			for(int j =0; j<first[i].length; j++)
			{
				first[i][j] = first[i][j] + ((double)second[i][j])*scaled;
			}
	}
	
	LeastSquaresInterpolation interpolation = new LeastSquaresInterpolation(2);
	
	private Result getInterpolatedMaximum(int minSensor, int maxSensor, double[] trackEdgeSensors) {
		double maxEdgeDistance = .1;
		int maxAngleIndex = Math.max(minSensor, 0);
		for (int i = Math.max(minSensor, 0); i <= Math.min(maxSensor, 18); i++) {
			double thisTrackEdgeSensor = trackEdgeSensors[i];
			if (maxEdgeDistance < thisTrackEdgeSensor) {
				maxEdgeDistance = thisTrackEdgeSensor;
				maxAngleIndex = i;
			}
		}
		int count=0;
		/*for(int i=0; i<2; i++) {
			int min = -1, max = -1;
			switch(count) {
			case 0: min = maxAngleIndex[0]; max = maxAngleIndex[0]; break;
			case 1: 
				if(maxAngleIndex[0] < maxAngleIndex[1]) {
					min = maxAngleIndex[0]; max=maxAngleIndex[1];
				}else{ min = maxAngleIndex[1]; max=maxAngleIndex[0]; }
				break;
			}
			count++;
			if(min==0) maxAngleIndex[count] = max+1;
			else if(max==18) maxAngleIndex[count] = min-1;
			else if(trackEdgeSensors[min-1] > trackEdgeSensors[max+1]) {
				maxAngleIndex[count] = min-1;
			}else{maxAngleIndex[count] = max+1;}
		}*/
		if(maxAngleIndex==0 || maxAngleIndex == 18) return new Result(maxAngleIndex, maxEdgeDistance);
		
		int[] maxAngleIndices = new int[3];
		maxAngleIndices[0]= maxAngleIndex;
		maxAngleIndices[1] = maxAngleIndex-1;
		maxAngleIndices[2] = maxAngleIndex+1;
		double[] distances = new double[3];
		for(int i=0; i<distances.length; i++)
			distances[i] = trackEdgeSensors[maxAngleIndices[i]];
		Result res = interpolation.putPoints(maxAngleIndices, distances);
		if(res.maxAngle<0) res.maxAngle=0;
		if(res.maxAngle>18) res.maxAngle=18;
		if(doShowOutputs) outputWindow.putData2(maxAngleIndices, distances, res.maxAngle, res.maxDistance);
		return res;
	}
	
	public Action control(SensorModel sensors)
	{
		Action action = new Action ();

		double desiredSteeringDirection = 0;
		double targetSpeed = 0;

		final double currentSpeed = sensors.getSpeed(); /* current speed of car */
		final double currentABSSpeed = Math.abs(currentSpeed); /* current speed of car */
		final double angleToTrack = sensors.getAngleToTrackAxis(); /* angle between the car direction and 
        	the direction of the track axis [-pi, pi] */
		final double trackPosition = sensors.getTrackPosition(); /* distance between car and track axis. Normalized wrt track width. -1=right edge, +1=left edge. */
		final double latSpeed = sensors.getLateralSpeed();
		final double trackLocation = sensors.getDistanceFromStartLine();
		final double distanceRaced = sensors.getDistanceRaced();
		
		stepCounter++;
		double[] trackEdgeSensors = useFilter ? edgeDistanceFilter.getNextEstimation(currentSpeed, latSpeed, sensors.getTrackEdgeSensors(), stepCounter, wasOffTrackCounter) : sensors.getTrackEdgeSensors();
//		for(int i=0; i<trackEdgeSensors.length; i++) {
//			if(trackEdgeSensors[i]<0) System.out.println("after filter: trackEdgeSensor["+i+"]: "+trackEdgeSensors[i]);
//		}
		
		
		final double[] opponents = sensors.getOpponentSensors();

		
		updateOpponents(filterOpponents(opponents));

		//######################################### Focus-Test #################################################
		action.focus = 0;//point the focus sensors straight ahead
		if(doSysouts && sensors.getFocusSensors()[2] > 0)//ML
			System.out.println("FocusReading[meters]: "+sensors.getFocusSensors()[2]+" with Angle[degrees]: "+focusAngle+" Time: "+sensors.getCurrentLapTime());//ML

//		if(sensors.getCurrentLapTime() > 10)
//			action.restartRace = true;
		
	/*	if(counter-->=0){
			File testfile = new File("c:\\temp\\streckenbreite1.txt");
			DecimalFormat df =   new DecimalFormat  ( ",##0.00" );
		    
			
			try {
				BufferedWriter fw = new BufferedWriter(new FileWriter(testfile,true));
	
	//			for(int i=0; i<19; i++){
			 
					fw.write(df.format(trackEdgeSensors[0]+trackEdgeSensors[trackEdgeSensors.length-1]) + System.getProperty("line.separator"));
					
		//			if(trackEdgeSensors[i] < )
		//			{
		//				negative += 1;
		//				System.out.println("negative: "+negative);
		//			}
	//			}
				fw.write("########" + System.getProperty("line.separator"));
				
				fw.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			
			}
		}*/
		//######################################################################################################

		if (trackWidth == 0.0)
		{				
			trackWidth = trackEdgeSensors[0] + trackEdgeSensors[18];
			if (trackWidth > 28.0 && doAdjustSpeedwaySteering == true)
			{
				onTrackValues.put("SteeringFactor", 0.1);
				angularAdjustments = .1;
				if(doSysouts) System.out.println("Am adjusting to speed-way steering....!");
			}
		}

		// #######################################################################################################
		// The following makes sure that the wasOffTrackCounter is enabled even if only one back tire is off-track:
		if(wasOffTrackCounter < 2) {
			x = d * Math.sin(angleToTrack + Math.asin(0.5*b / d));

			if(Math.abs(trackPosition) > 1. - ((x - 0.5*b) / (0.5*trackWidth)) &&
					((angleToTrack > 0 && trackPosition > 0) ||
							(angleToTrack < 0 && trackPosition < 0))) {
				wasOffTrackCounter = 5;
				if(doSysouts)
					System.out.println("DANGER! Your rear is off-track! "+(1. - ((x - 0.5*b) / (0.5*trackWidth))));
			} 
		}
		// #######################################################################################################


		if (currentABSSpeed > offTrackValues.get("StuckSpeed"))
			stuckCounter = 0;
		else{
			stuckCounter++;
		}

		if (behavior == Behavior.NORMAL &&
				(stuckCounter > offTrackValues.get("StuckCounterMax") && distanceRaced > 50. && sensors.getRPM() < 1000) 
				|| (doDemolitionDerby && stuckCounter > 100 && sensors.getCurrentLapTime() > 5 && sensors.getRPM() < 1300))
		{
			stuckAngle = angleToTrack;
			stuckCounter = 0;
			behavior = Behavior.REVERSE;
			if(doSysouts)
				System.out.println("STUCK with V:"+currentABSSpeed);
			stuck2Counter = -500;//disable stuck2 detection for the next 10 seconds
			reverseCounter = 0;
		}
		if (behavior == Behavior.REVERSE &&	Math.abs(angleToTrack / stuckAngle) < .5 && !stalled)
		{
			behavior = Behavior.BRAKE;
		}
		if (behavior == Behavior.REVERSE && stuckCounter > offTrackValues.get("StuckCounterMax") && !stalled)
		{
			stuckCounter = 0;
			behavior = Behavior.NORMAL; 
		}
		if (behavior == Behavior.BRAKE &&
				currentABSSpeed < offTrackValues.get("StuckSpeed"))
		{
			behavior = Behavior.NORMAL;
		}
		if(behavior==Behavior.REVERSE || stalled){
			reverseCounter++;
			if(reverseCounter > 100){
				reverseCounter = 0;
				behavior = Behavior.NORMAL;
				stalled = false;
			}
		}


		// ############################################# Stuck2-Detection ########################################################
		// A second stuck-detection, this time for situations in which the car's front is scraping along a wall and 
		// cannot be turned back towards the track by further accelerating---thus, the use of the reverse gear is required. 
		// (in such situations, the first stuck-detection does not trigger because of the higher speed of the car)
		// A stuck2-situation is detected if the car is off-track, turned away from the track, and has a speed between 5 and 50. 
		// These criteria have to be fulfilled for a certain period of time that is longer for higher speeds (which have a
		// better chance of freeing themselves after a while) and shorter for slower speeds (which have higher priority since
		// more time is lost). To prevent circular behavior (e.g., stuck2 -->reverse -->normal -->stuck2; especially possible when
		// being still off-track after having driven backwards, because of the slower off-track acceleration) the stuck2-detection
		// is disabled after any stuck situation for the next 500 time steps (see also above, in the original stuck-detection), 
		// which is about 10 seconds.
		if(doStuck2Detection) {
			if (currentSpeed > 50) {
				stuck2Counter = 0;
			}
			else if(stuck2Counter >= 0) {
				if(currentSpeed > 5 && wasOffTrackCounter > 0 &&
						((angleToTrack < -Math.PI/8. && trackPosition > 1)) ||
						(angleToTrack > Math.PI/8. && trackPosition < -1)) {
					stuck2Counter += 5 - (int)(currentABSSpeed / 10);
				}
				else {
					stuck2Counter = 0;
				}
			}
			else {
				stuck2Counter++; 
			}

			if (behavior == Behavior.NORMAL && stuck2Counter > 600 && distanceRaced > 150.)	{
				stuckAngle = angleToTrack;
				behavior = Behavior.REVERSE;
				if(doSysouts) System.out.println("STUCK2 with V:"+currentSpeed);
				stuck2Counter = -500;//disable stuck2 detection for the next 10 seconds
			}
			if (behavior == Behavior.REVERSE &&	Math.abs(sensors.getTrackPosition()) < 0.5)	{
				behavior = Behavior.BRAKE;//stop reverse behavior if you are back on the track
			}
		}
		// #######################################################################################################################



		// setting targetSpeed and desiredDirection...
		if (Math.abs(trackPosition) >= 1.0)
		{ 	// off track - trying to get back on.
			targetSpeed = offTrackValues.get("TargetSpeed"); // fixed target speed off track - could be evolved...

			double targetAngle = (trackPosition / Math.abs(trackPosition) 
					* (Math.abs(trackPosition)-offTrackValues.get("InnerTrackBorder")) 
					* offTrackValues.get("TrackBorderDirectionFactor"));
			desiredSteeringDirection = angleToTrack - targetAngle;

			// increase speed if we are heading the right direction 
			targetSpeed += offTrackValues.get("TargetSpeedFactor") * Math.max(0,  1. - offTrackValues.get("DirectionSpeedInfluence") * Math.abs(desiredSteeringDirection)); 

			wasOffTrackCounter = wasOffTrackIntegration;
		}
		else
		{        
			// get track edge sensors
			//double[] trackEdgeSensors = sensors.getTrackEdgeSensors(); /* range finder sensors [0,100] */
			sensors.getSpeed();
			
			if(avoidOpponents && currentSpeed != 0) {

				for (Car opponent : opponentData)
				{
					if(!opponent.isVisible() && opponent.getUnseenCounter()>2)
						continue;
					double relVel = -opponent.getRelativeVelocity() / tickTime;
					if (relVel <= 0) continue;
//					double tOpponentCrash = (opponent.getDistance() - carLength) / relVel;
					//don't use relative velocities
					double tOpponentCrash;
					if (opponent.getDistance()<sophisticatedAvoidanceDistance)
						tOpponentCrash = (opponent.getDistance() - carLength) / currentSpeed;
					else continue;
					
					int sector = opponent.getSector();

					if (sector < 9 || sector > 26) continue;

					int leftEdgeIndex = opponent.getLeftCorrespondingTrackSensor();
					double tCrashLeftEdge = trackEdgeSensors[leftEdgeIndex] * 3.6 / currentSpeed;
					if (tOpponentCrash < tCrashLeftEdge)
					{
						//	        		if(doSysouts) System.out.println("leftEdgeIndex: " + leftEdgeIndex + "; Sensor alt: " + trackEdgeSensors[leftEdgeIndex] + "; Sensor neu: " + tCrash * currentSpeed / 3.6);
						trackEdgeSensors[leftEdgeIndex] = Math.max(1.0, tOpponentCrash * currentSpeed / 3.6);
					}

					int rightEdgeIndex = opponent.getRightCorrespondingTrackSensor();
					double tCrashRightEdge = trackEdgeSensors[rightEdgeIndex] * 3.6 / currentSpeed;
					if (tOpponentCrash < tCrashRightEdge)
					{
						//	        		if(doSysouts) System.out.println("rightEdgeIndex: " + rightEdgeIndex + "; Sensor alt: " + trackEdgeSensors[rightEdgeIndex] + "; Sensor neu: " + tCrash * currentSpeed / 3.6);
						trackEdgeSensors[rightEdgeIndex] = Math.max(1.0, tOpponentCrash * currentSpeed / 3.6);
					}

				}
			}

			// only consider the ones that point towards the front
			int minSensor = (int) Math.ceil(-angleToTrack/Math.PI * 18);
			int maxSensor = (int) Math.ceil((1 - angleToTrack/Math.PI) * 18);

			// determine maximal track sensor value and angle of maximal track sensor
			double maxEdgeDistance = .1;
			double maxAngle;
			int maxAngleIndex = Math.max(minSensor, 0);
			
			if(!useInterpolation) {
				for (int i = Math.max(minSensor, 0); i <= Math.min(maxSensor, 18); i++)
				{
					double thisTrackEdgeSensor = trackEdgeSensors[i];
					if (maxEdgeDistance < thisTrackEdgeSensor)
					{
						maxEdgeDistance = thisTrackEdgeSensor;
						maxAngleIndex = i;
					}
				}
				
				if(maxAngleIndex > 0 && maxAngleIndex < 18) {
					double diffLeft = maxEdgeDistance - trackEdgeSensors[maxAngleIndex-1];
					double diffRight = maxEdgeDistance - trackEdgeSensors[maxAngleIndex+1];
					maxAngle = maxAngleIndex - angularAdjustments + 
					2. * angularAdjustments * (diffLeft / (diffLeft+diffRight));
				}else
				if(maxAngleIndex == 0)
				{
					double diffRight = maxEdgeDistance - trackEdgeSensors[maxAngleIndex+1];
					maxAngle = maxAngleIndex - angularAdjustments + 
					2. * angularAdjustments * (maxEdgeDistance / (maxEdgeDistance+diffRight));
				}
				else if(maxAngleIndex == 18)
				{
					double diffLeft = maxEdgeDistance - trackEdgeSensors[maxAngleIndex-1];
					maxAngle = maxAngleIndex - angularAdjustments + 
					2. * angularAdjustments * (diffLeft / (diffLeft + maxEdgeDistance));	        	
				}
				else
				{
					maxAngle = maxAngleIndex;
				}
			}else {//useInterpolation
				Result res = getInterpolatedMaximum(minSensor, maxSensor, trackEdgeSensors);
				maxAngle = res.maxAngle;
				maxEdgeDistance = res.maxDistance;
			}

			if(doDemolitionDerby){//Overwrite regular steering direction for demolition derby behavior:
				for (Car opponent : opponentData){
					//If you see an opponent, drive into him:
					if(!opponent.isVisible() && opponent.getUnseenCounter()>2)
						continue;
					maxAngle = opponent.getSector() - 8.5;
					maxAngle = Math.max(0, maxAngle);
					maxAngle = Math.min(18, maxAngle);
					//Try to turn if he is behind you:
					if(maxAngle==0 && trackEdgeSensors[0]<20){
						maxAngle = 18;
					}else if(maxAngle==18 && trackEdgeSensors[18]<20){
						maxAngle = 0;
					}
					//Switch back to normal driving behavior in order to drive around the island in
					//the track's middle, if the opponent is in front of you but behind a wall:
					if(trackEdgeSensors[(int) maxAngle] < opponent.getDistance()-10 &&
							maxAngle > 0 && maxAngle < 18 && normalCounter < 20){
						normalCounter++;//count the time of regular behavior
						if(normalCounter==19)
							normalCounter = 50;
						maxAngle = maxAngleIndex;			
						continue;
					}
					//Don't crash into a wall without a chance of harming your opponent:
					if((trackEdgeSensors[9]<10 && (trackEdgeSensors[8]<5 || trackEdgeSensors[10]<5)) 
							&& normalCounter < 20 && (maxAngle < 5 || maxAngle > 13) && currentSpeed > 10){ 
						normalCounter++;
						if(normalCounter==19)
							normalCounter = 50;
						maxAngle = maxAngleIndex;			
						continue;
					}
					//If he is far behind you make a u-turn towards the side with more space:
					if(maxAngle==0 && trackEdgeSensors[0]<trackEdgeSensors[18] && opponent.getDistance()>50 
							&& turnOver==0 && trackEdgeSensors[0]<30){
						turnOver++;
						maxAngle = 18;
					}else if(maxAngle==18 && trackEdgeSensors[0]>trackEdgeSensors[18] && opponent.getDistance()>50 
							&& turnOver==0 && trackEdgeSensors[18]<30){
						turnOver--;
						maxAngle = 0;
					}else if(turnOver != 0 && maxAngle > 0 && maxAngle < 18){//stop u-turning if he is in front of you
						turnOver = 0;
					}else if(turnOver > 0){
						maxAngle = 18;
						turnOver++;
					}else if(turnOver < 0){
						maxAngle = 0;
						turnOver--;
					}
					if(Math.abs(turnOver)>300)//stop u-turning after a certain period of time
						turnOver = 0;
					//If he got you stalled by pushing you backwards, try to free yourself with the reverse gear:
					if(opponent.getDistance() < 5 && opponent.getSector()-9 > 7 && opponent.getSector()-9 < 11
							&& sensors.getRPM() < 1300 && sensors.getGear()==1 && reverseCounter==0){
						stalled = true;
					}else if(stalled)//if pushed backwards, turn your wheels to free yourself:
						if(opponent.getSector()-8.5 > 9)
							maxAngle = 18;
						else
							maxAngle = 0;

					if(normalCounter > 0)
						normalCounter--;//return to demolition behavior after a certain time of normal behavior
				}
			}

			// decide how to steer
			desiredSteeringDirection = (9. - maxAngle) * onTrackValues.get("SteeringFactor");


			// now:
			// maxEdgeDistance is the distance of the largest edge sensor
			// maxEdgeAngle is the orientation of that sensor (1=all to the left)

			// First, determine the target speed out of this information

			// target speed based on distance sensors
			targetSpeed =
				onTrackValues.get("SpeedOffset") +
				onTrackValues.get("SpeedLinearScale") * maxEdgeDistance +
				onTrackValues.get("SpeedPolynomialScale") * Math.pow(
						Math.max(0, maxEdgeDistance-onTrackValues.get("SpeedDistanceThreshold")) /
						(onTrackValues.get("SpeedMaximumDistance") - onTrackValues.get("SpeedDistanceThreshold")),
						onTrackValues.get("SpeedDistancePow")) -
						onTrackValues.get("SpeedAngleDecreaseScale") * Math.pow(
								(Math.abs(maxAngle-9)/9.), onTrackValues.get("SpeedAnglePow")) +
								(maxEdgeDistance > onTrackValues.get("SpeedMaximumDistance") ? 500 : 0);

			if(offRoadTrack) {
				if(latSpeed * desiredSteeringDirection > 0)
					targetSpeed -= onTrackValues.get("dirtTrackLateralSpeedDecrease") * 
					latSpeed * desiredSteeringDirection;
				wasOffTrackCounter = 0;
			}else if(wasOffTrackCounter>0) {
				targetSpeed = (offTrackValues.get("TargetSpeed") * (double)wasOffTrackCounter + 
						targetSpeed * (double)(wasOffTrackIntegration-wasOffTrackCounter))/
						(double)wasOffTrackIntegration;
				wasOffTrackCounter --;
			}

			targetSpeed = Math.max(onTrackValues.get("SpeedMinimum"), targetSpeed);

			double minOpponentDistance = getOpponentMinValue(opponents, maxAngleIndex-9, 5.);
			if(doShowOutputs) outputWindow.putOpponentDistance(minOpponentDistance);
			if (minOpponentDistance < simpleAvoidanceDistance && doSimpleAvoidance) {
				if(doShowOutputs) outputWindow.putOpponentAvoidance(true);
//				targetSpeed *= Math.max(0, minOpponentDistance-5) / 10;
				double test = targetSpeed;
				targetSpeed = Math.min(targetSpeed, Math.pow(Math.max(0, minOpponentDistance-5), 1.3));
				if(doShowOutputs) outputWindow.putTargetSpeed(test, targetSpeed);
				
			}else if(doShowOutputs) outputWindow.putOpponentAvoidance(false);

			//Decrease speed when turning without enough space in demolition mode:
			if(doDemolitionDerby){
				if(maxAngle==0 || maxAngle==18)
					targetSpeed = 30 + trackEdgeSensors[9];
				if(stalled)
					targetSpeed = 100;
			}

		}

		// decide whether to accelerate based on determined target speed
		if (currentSpeed < targetSpeed)
		{
			// could be improved ... is a turn approaching or are we far from that?
			action.accelerate = 1.0;
		}
		else
		{
			// decide whether to brake
			if (currentSpeed > targetSpeed * onTrackValues.get("BrakeLevel"))
			{
				action.brake = Math.min(1.0, onTrackValues.get("BrakeDegree") *
						(currentSpeed - targetSpeed * onTrackValues.get("BrakeLevel")));
			}
		}

		if(offRoadTrack) {
			// decreasing maximum acceleration value if applicable
			action.accelerate = Math.min(action.accelerate, onTrackValues.get("dirtTrackLateralAccelerationDecrease"));
		}

		action.steering = (currentSpeed < 0 ? -1 : 1) * desiredSteeringDirection;

		/* ###########################################################
		 * ############################# gear shifting ################
		 * ############################################################*/

		final double rpm = sensors.getRPM();

		if (sensors.getCurrentLapTime() < 0)
		{
			; // do nothing if not yet started...
		}
		else if (behavior == Behavior.REVERSE)
		{
			gear = -1;
		}
		else if (behavior == Behavior.BRAKE)
		{
			gear = 0;
		}
		else if (waitForGearSwitch > 0)
		{
			waitForGearSwitch--;
		}
		else
		{
			// if gear is 0 (N) or -1 (R) just return 1
			if (gear < 1)
			{
				gear = 1;
				waitForGearSwitch = 25;
			}
			// check if the RPM value of car is greater than the one suggested
			// to shift up the gear from the current one
			else if (gear < 6 && rpm >= gearUp[gear-1])
			{
				gear++;
				waitForGearSwitch = 25;
			}
			// check if the RPM value of car is lower than the one suggested
			// to shift down the gear from the current one
			else if (gear > 1 && rpm <= gearDown[gear-1])
			{
				gear--;
				waitForGearSwitch = 25;
			}
		}
		action.gear = gear;
		if(stalled)//If in demolition derby the opponent pushes you backwards, switch to reverse gear:
			action.gear = -1;

		// ----------- ABS/ASR -----------
		// compute the speed of wheels in m/s
		double slip = .0;
		for (int i = 0; i < 4; i++)
		{
			slip += sensors.getWheelSpinVelocity()[i] * wheelRadius[i];
		}
		// slip is the difference between actual speed of car and average speed of wheels
		slip = currentSpeed - slip/4.0*3.6;

		double slipF = .0;
		double slipR = .0;
		slipF = (sensors.getWheelSpinVelocity()[0]+
				sensors.getWheelSpinVelocity()[1])
				* wheelRadius[0];

		slipR = (sensors.getWheelSpinVelocity()[2]+
				sensors.getWheelSpinVelocity()[3])
				* wheelRadius[2];
		slipF = currentSpeed - slipF/2.0*3.6;
		slipR = currentSpeed - slipR/2.0*3.6;

		// when speed lower than min speed for abs do nothing
		if (currentSpeed > absMinSpeed)
		{
			// when slip too high apply ABS
			if (slip > onTrackValues.get("ABSSlip"))
			{
				action.brake = Math.max(0, action.brake - (slip - onTrackValues.get("ABSSlip"))/onTrackValues.get("ABSRange"));
			}
		}

		// ----------- ASR -----------
		if (offRoadTrack == true && doDetectOffroadTrack == true)
		{
			// when negative slip too high apply dirt-track ASR
			if (-slipR/2. > offRoadASRValues.get("ASRSlip"))
			{
				action.accelerate = Math.max(0, action.accelerate + 
						(slipR/2. + offRoadASRValues.get("ASRSlip"))/offRoadASRValues.get("ASRRange"));
			}
		}
		else if (Math.abs(trackPosition) >= 1.0)
		{
			// when negative slip too high apply ASR
			if (-slipR/2. > ASRValues.get("ASRSlip"))
			{
				action.accelerate = Math.max(0, action.accelerate + 
						(slipR/2. + ASRValues.get("ASRSlip"))/ASRValues.get("ASRRange"));
			}
		}


		// ############################################## Jump Detection: ######################################################
		// Prevent loss of control when landing after a jump by detecting jumps and adjusting the angle of the front wheels to
		// the angle that the car is flying with until a landing is detected. Jumps are detected when the rear slip is huge (no
		// contact to the ground), lateral speed is low (forward movement), and forward speed is higher than 50km/h (to prevent
		// false jump detection when accelerating from the start line with high rear slip). Landings are detected when the front
		// wheels are greatly understeering (due to the sudden contact with the ground with high velocity). As a fail-safe, 
		// landings are detected also after 1 second after a detected jump or if the car accelerates after a jump (which is an
		// indicator of ground contact).
		if(doJumpDetection) {
			// Detect jumps:
			if(jump==false && Math.abs(latSpeed) < 5. && slipR < -50 && currentSpeed > 50) 
				jumpDetection++;
			else
				jumpDetection = 0;

			if(jumpDetection > 2) {
				if(doSysouts) System.out.println("JUMP detected! Watch for birds, you are flying! T:"+sensors.getCurrentLapTime()+" V:"+currentSpeed);
				jump = true;
				jumpTime = sensors.getCurrentLapTime();
				jumpSpeed = currentSpeed;
			}
			// Detect landings:
			if(jump==true && slipF > 15) {
				if(doSysouts) System.out.println("LANDING detected! Welcome back on earth! T:"+sensors.getCurrentLapTime()+" V:"+currentSpeed);
				jump = false;
			}else if(jump==true && jumpTime+1. < sensors.getCurrentLapTime()) {
				if(doSysouts) System.out.println("FORCED SWITCH TO LANDING! You can't be flying this long! T:"+sensors.getCurrentLapTime()+" V:"+currentSpeed);
				jump = false;
			}else if(jump==true && currentSpeed > jumpSpeed+6) {
				if(doSysouts) System.out.println("FORCED SWITCH TO LANDING! Seems like false alarm! T:"+sensors.getCurrentLapTime()+" V:"+currentSpeed);
				jump = false;
			}
			// Show appropriate reaction for successful landing:
			if(jump) {
				action.steering = Math.asin(latSpeed/Math.sqrt(Math.pow(latSpeed,2)+Math.pow(currentSpeed,2))) / 0.785398;
				if(action.steering < -1)	action.steering = -1;
				else if(action.steering > 1)	action.steering = 1;
			}
		}	
		// #####################################################################################################################



		// #################################### Off-Track Speed Reduction: ##########################################
		// Prevent rear wheel burn-outs and resulting loss of control when off-track by reducing acceleration.
		// Acceleration is decreased by initially 90%. This value is further adapted to keep the rear slip below -15
		// by slowly decreasing the speed reduction and quickly increasing it when the rear wheels slip again. 
		final double minSpeedReduction = 0.1;
		if(doSpeedReduction) 
			// decrease acceleration when off-road:
			if(doSpeedReductionOnTrack || wasOffTrackCounter > 0) {
				if(!doSpeedReductionOnTrack && offTrackSpeedReduction==false) {
					speedReduction = 9;
					offTrackSpeedReduction = true;
				}
				if(slipR < -15) {
					speedReduction += 0.5*(speedReduction < minSpeedReduction ? minSpeedReduction : speedReduction);
				}
				else
					speedReduction -= speedReduction * 0.1;
				if(speedReduction > 10)
					speedReduction = 10;
				else if(speedReduction < 0)
					speedReduction = 0;
				action.accelerate *= 1 - (0.1*speedReduction);
				//				action.accelerate *= 0.01;
				//				action.brake *= 0.01;
			}
			else {
				speedReduction = 0;
				offTrackSpeedReduction = false;
			}
		// ##########################################################################################################


		// ####################################################
		// crash prevention in subsequent rounds.... 
		// ####################################################
		if(doCrashLocationPrevention) {
			crashLocationPreventer.notifyCrashLocator(sensors, targetSpeed, 
					action.accelerate, action.brake);

			// add crash point speed acceleration / break adjustments here
			if(crashLocationPreventer.checkCrashLocations(trackLocation) ) {
				//if(doSysouts) System.out.println("Inside danger zone... ");
				// inside a dangerous area... 
				double targetCrashPreventionSpeed = 
					crashLocationPreventer.getCrashPreventionTargetSpeed();
				if(targetCrashPreventionSpeed > 0) {
					if(currentSpeed > targetCrashPreventionSpeed) {
						action.accelerate = 0;
						action.brake = Math.min(1., 
								(currentSpeed - targetCrashPreventionSpeed) / 10.);
					}
				}
			}
		}
		// ##########################################################################################################


		// ----------- OffRoad track detector -----------

		if (distanceRaced < 100 && doDetectOffroadTrack && offRoadTrack == false) 
		{
			if(!offRoadTrack && currentSpeed < 31. && -slipR > 20)
			{
				System.out.println("Switch to DIRTTRACK due to " +
						"DR: "+distanceRaced+
						" vel: "+currentSpeed+
						" acc: "+action.accelerate + 
						" gear: "+action.gear + 
						" => slipR: "+(-slipR)+ "(slip="+(-slip)+")");
				if(doSysouts) System.out.println("Wheel velocity: 0:"+sensors.getWheelSpinVelocity()[0]+
						" 1:"+ sensors.getWheelSpinVelocity()[1]+
						" 2:"+sensors.getWheelSpinVelocity()[2]+
						" 3:"+sensors.getWheelSpinVelocity()[3]);
				offRoadTrack = true;
				switchDrivingStyle(DrivingStyle.DIRTTRACK);
			}
		}

		if (behavior == Behavior.BRAKE || doRadicalBreak)
		{
			action.brake = 1;
			action.accelerate = 0;
		}

		if (doDeliberateCrashing)
		{
			// ----------- Crash generator -----------
			if (distanceRaced > crashCounter * crashDistance)
			{
				crashCounter++;
				crashActiveCounter = crashDuration;
			}

			if (crashActiveCounter > 0)
			{
				action.steering = (crashCounter%2 == 0) ? 1 : -1;
				crashActiveCounter--;
			}
		}

		if (distanceRaced > 200)
		{
			if (speedMin == 0.0)
				speedMin = currentSpeed;
			else
				if (speedMin > currentSpeed)
					speedMin = currentSpeed;
			if (speedMax < currentSpeed)
				speedMax = currentSpeed;
		}

		if (firstRoundCompleted == false && sensors.getLastLapTime() > 0.0)
		{// end of first round - let's see if we switch strategies...
			double avgSpeed = distanceRaced / sensors.getLastLapTime();

			if(!offRoadTrack && doSwitchDrivingStyle == true && 
					(!doCrashLocationPrevention || 
							crashLocationPreventer.getNumCrashPositions()==0) ) {
				if (avgSpeed < 37) { // Previously: 43
					if(doSysouts) System.out.println("Switch to SLOW driving style due to average speed "+avgSpeed);
					switchDrivingStyle(DrivingStyle.SLOW);
				}else if (speedMin > 70) {
					if(doSysouts) System.out.println("Switch to FAST driving style due to minimum speed "+speedMin);
					switchDrivingStyle(DrivingStyle.FAST);
				}
			}
			firstRoundCompleted = true;
		}

//		out("trackEdgeSensors for visualization", trackEdgeSensors);
//		for(int i=0; i<trackEdgeSensors.length; i++) {
//			if(trackEdgeSensors[i]<0) System.out.println("trackEdgeSensor["+i+"]: "+trackEdgeSensors[i]);
//		}
		if(doShowTrackMonitor) {	
			monitor.visualizeSensors(trackEdgeSensors, opponentData, focusAngle, sensors.getFocusSensors());
		}

		if(doSteerLeft)
			action.steering = -1;

		return action;
	}

	/**
	 * Returns the minimal value in the sensor array, which does not exceed width
	 * 
	 * @param val Double array of values.
	 * @param width Double value representing width of sensor range.
	 * @return
	 */
	private double getOpponentMinValue(double[] val, int direction, double width) 
	{
		double ret = 1000;
		for(int i = 17+direction; i <= 18+direction; i++) {
			if(val[i]<ret) // && width > Math.sin((i*10-175)/180 * Math.PI) * val[i])
			{
				ret = val[i];
			}
		}
		return (ret<0 ? 1 : ret);
	}

	private void switchDrivingStyle(DrivingStyle style)
	{
		switch (style)
		{
		case SLOWEST:
			// Best of O04 (Track: Aalborg)
			onTrackValues.put("SpeedOffset", -6.002914321065081);
			onTrackValues.put("SpeedLinearScale", 2.729906793550739);
			onTrackValues.put("SpeedPolynomialScale", 303.4803309481403);
			onTrackValues.put("SpeedDistanceThreshold", 16.7183021870276);
			onTrackValues.put("SpeedDistancePow", 7.659681502033434);
			onTrackValues.put("SpeedAngleDecreaseScale", 234.93266853280483);
			onTrackValues.put("SpeedAnglePow", 3.3289841925411574);
			onTrackValues.put("SpeedMaximumDistance", 98.56523135789904);
			onTrackValues.put("SpeedMinimum", 18.802220107958394);
			onTrackValues.put("BrakeLevel", 1.2675520816337558);
			onTrackValues.put("BrakeDegree", 1.0666299158092516);
			onTrackValues.put("ABSSlip", 22.223740019646893);
			onTrackValues.put("ABSRange", 15.940527842921721);
			onTrackValues.put("SteeringFactor", 1.8632095257047225);
			onTrackValues.put("dirtTrackLateralSpeedDecrease", 0.);
			onTrackValues.put("dirtTrackLateralAccelerationDecrease", 1.);
			break;
		case DIRTTRACK:
			onTrackValues.put("SpeedOffset", 13.018697650479941);
			onTrackValues.put("SpeedLinearScale", 2.112185816188859);
			onTrackValues.put("SpeedPolynomialScale", 162.4715784351532);
			onTrackValues.put("SpeedDistanceThreshold", 29.27106264697025);
			onTrackValues.put("SpeedDistancePow", 11.191935813893727);
			onTrackValues.put("SpeedAngleDecreaseScale", 1241.0106721250204);
			onTrackValues.put("SpeedAnglePow", 2.942415608704355);
			onTrackValues.put("SpeedMaximumDistance", 77.2702181707971);
			onTrackValues.put("SpeedMinimum", 34.24832150215329);
			onTrackValues.put("BrakeLevel", 1.283772711870475);
			onTrackValues.put("BrakeDegree", 0.849195780683764);
			onTrackValues.put("ABSSlip", 12.408582123810419);
			onTrackValues.put("ABSRange", 3.5112558890701937);
			onTrackValues.put("SteeringFactor", 0.3); // 0.6351626237488357
			onTrackValues.put("dirtTrackLateralSpeedDecrease", 1.5883586803451402);
			onTrackValues.put("dirtTrackLateralAccelerationDecrease", 0.8437255450523989);
			break;
		case SLOW:
			// Best of O18 (Track: Spring)
			onTrackValues.put("SpeedOffset", 18.22);
			onTrackValues.put("SpeedLinearScale", 2.24);
			onTrackValues.put("SpeedPolynomialScale", 168.91);
			onTrackValues.put("SpeedDistanceThreshold", 0.19);
			onTrackValues.put("SpeedDistancePow", 9.30);
			onTrackValues.put("SpeedAngleDecreaseScale", 1147.75);
			onTrackValues.put("SpeedAnglePow", 1.92);
			onTrackValues.put("SpeedMaximumDistance", 84.80);
			onTrackValues.put("SpeedMinimum", 20.92);
			onTrackValues.put("BrakeLevel", 1.19);
			onTrackValues.put("BrakeDegree", 1.07);
			onTrackValues.put("ABSSlip", 7.76);
			onTrackValues.put("ABSRange", 2.79);
			onTrackValues.put("SteeringFactor", 0.46);
			onTrackValues.put("dirtTrackLateralSpeedDecrease", 0.);
			onTrackValues.put("dirtTrackLateralAccelerationDecrease", 1.);
			break;
		case NORMAL:
			// STANDARD
			// Best of O10 (Track: E-Track 6)
			onTrackValues.put("SpeedOffset", 43.23);
			onTrackValues.put("SpeedLinearScale", 1.99);
			onTrackValues.put("SpeedPolynomialScale", 104.76);
			onTrackValues.put("SpeedDistanceThreshold", 36.50);
			onTrackValues.put("SpeedDistancePow", 9.38);
			onTrackValues.put("SpeedAngleDecreaseScale", 907.68);
			onTrackValues.put("SpeedAnglePow", 1.92);
			onTrackValues.put("SpeedMaximumDistance", 97.33);
			onTrackValues.put("SpeedMinimum", 11.89);
			onTrackValues.put("BrakeLevel", 1.13);
			onTrackValues.put("BrakeDegree", 0.70);
			onTrackValues.put("ABSSlip", 11.7);
			onTrackValues.put("ABSRange", 10.18);
			onTrackValues.put("SteeringFactor", 0.39);
			onTrackValues.put("dirtTrackLateralSpeedDecrease", 0.);
			onTrackValues.put("dirtTrackLateralAccelerationDecrease", 1.);
			break;
		case FAST:
			// Best of O03 (Track: Wheel 1)
			onTrackValues.put("SpeedOffset", 8.64);
			onTrackValues.put("SpeedLinearScale", 1.48);
			onTrackValues.put("SpeedPolynomialScale", 118.38);
			onTrackValues.put("SpeedDistanceThreshold", -43.78);
			onTrackValues.put("SpeedDistancePow", 1.19);
			onTrackValues.put("SpeedAngleDecreaseScale", 558.62);
			onTrackValues.put("SpeedAnglePow", 1.27);
			onTrackValues.put("SpeedMaximumDistance", 94.28);
			onTrackValues.put("SpeedMinimum", 23.16);
			onTrackValues.put("BrakeLevel", 1.10);
			onTrackValues.put("BrakeDegree", 0.56);
			onTrackValues.put("ABSSlip", 5.49);
			onTrackValues.put("ABSRange", 2.01);
			onTrackValues.put("SteeringFactor", 0.37);
			onTrackValues.put("dirtTrackLateralSpeedDecrease", 0.);
			onTrackValues.put("dirtTrackLateralAccelerationDecrease", 1.);
			break;
		}
		if (trackWidth > 28.0 && doAdjustSpeedwaySteering == true)
		{
			onTrackValues.put("SteeringFactor", 0.1);
		}
		onTrackValues.put("SpeedOffset", 10.22);
		onTrackValues.put("SpeedLinearScale", 1.5);
	}

	public void reset()
	{
		gear = 1;
		waitForGearSwitch = 10;
		behavior = Behavior.NORMAL;
		stuckCounter = 0;
		stuck2Counter = 0;
		crashCounter = 1;
		crashActiveCounter = 0;
		offRoadTrack = false;
		firstRoundCompleted = false;
		speedMin = 0.0;
		speedMax = 0.0;
		if(doSpeedReductionOnTrack)
			switchDrivingStyle(DrivingStyle.DIRTTRACK);
		else switchDrivingStyle(DrivingStyle.NORMAL);
		opponentData.clear();
		trackWidth = 0.0;
		angularAdjustments = .5;
	}

	public void shutdown()
	{
		if(doSysouts) System.out.println("Bye bye!");		
	}

	public String toString()
	{
		return ""+evolvedValues;
	}

	public void setParameters(double[] parameters)
	{
		if (parameters.length != evolvedValues.size())
		{
			return;
		}
		Iterator<String> valueIterator = evolvedValues.keySet().iterator();
		int i = 0;
		while (valueIterator.hasNext())
		{
			String key = (String) valueIterator.next();
			evolvedValues.put(key, parameters[i++]);
		}
	}

	public boolean isFeasible ()
	{
		if (onTrackValues.get("SpeedDistanceThreshold") >= 100
				|| onTrackValues.get("BrakeLevel") < 0
				|| onTrackValues.get("BrakeDegree") < 0
				|| onTrackValues.get("SpeedMaximumDistance") > 101
				|| offTrackValues.get("TrackBorderDirectionFactor") < 0
				|| offTrackValues.get("StuckSpeed") < 0)
			return false;
		else
			return true;
	}

	public double[] getParameters()
	{
		double[] parameters = new double[evolvedValues.size()];
		Iterator<String> valueIterator = evolvedValues.keySet().iterator();
		int i = 0;
		while (valueIterator.hasNext())
		{
			String key = (String) valueIterator.next();
			parameters[i++] = evolvedValues.get(key);
		}
		return parameters;
	}

	public double[] getInitialDeviationParameters()
	{
		return initialDeviationParameters;
	}

	public double getClosestOpponentDistance() 
	{
		double dis = 100;
		if(opponentData==null)
			return dis;
		for (Car opp : opponentData)
		{
			if(opp.getDistance() < dis)
				dis = opp.getDistance();
		}
		return dis;
	}

	/**
	 * Updates the opponents that are currently perceived around the car. 
	 * 
	 * @param opponents The opponents as Car objects.
	 */
	@SuppressWarnings("unchecked")
	public void updateOpponents(double[] opponents)
	{
		Vector<Car> previousOpponents = (Vector<Car>)opponentData.clone();

		// first add all currently viewable cars to vector currentOpponents
		Vector<Car> currentOpponents = new Vector<Car>();
		for (int i=0; i<36; i++)
		{
			if (opponents[i] < 200 && !doDemolitionDerby)
			{
				currentOpponents.add(new Car(i, opponents[i]));
			}
			else if (opponents[i] < 300 && doDemolitionDerby)//ML
			{
				currentOpponents.add(new Car(i, opponents[i]));
			}
		}

		// now compare currently viewable cars with the cars in the history data
		while (currentOpponents.size() > 0 && previousOpponents.size() > 0)
		{
			// opponents from last iteration (best matching one to currently seens ones)
			Car bestPrevOppMatching = previousOpponents.firstElement();
			// currently seen opponent
			Car bestViewableOppMatching = currentOpponents.firstElement();

			double minDist = bestPrevOppMatching.distanceToExpetation(bestViewableOppMatching);

			for (Car prevOpponent : previousOpponents)
			{
				if (prevOpponent.distanceToExpetation(bestViewableOppMatching) < minDist)
				{
					bestPrevOppMatching = prevOpponent;
					minDist = prevOpponent.distanceToExpetation(bestViewableOppMatching);
				}
			}

			// bestPrevOppMatching is now the closest previous opponent to the
			// viewable "bestViewableOppMatching" opponent with a 
			// distance of "minDist"

			// now check the other way round... 
			double minDistOld;
			do
			{
				minDistOld = minDist;
				// if there is another viewable opponent that is even closer to the 
				// selected previous one... then change things... 
				for (Car currOpponent : currentOpponents)
				{
					if (bestPrevOppMatching.distanceToExpetation(currOpponent) < minDist)
					{
						bestViewableOppMatching = currOpponent;
						minDist = bestPrevOppMatching.distanceToExpetation(currOpponent);
					}
				}

				for (Car prevOpponent : previousOpponents)
				{
					if (prevOpponent.distanceToExpetation(bestViewableOppMatching) < minDist)
					{
						bestPrevOppMatching = prevOpponent;
						minDist = prevOpponent.distanceToExpetation(bestViewableOppMatching);
					}
				}
			}
			while (minDist < minDistOld);

			bestPrevOppMatching.update(bestViewableOppMatching);

			currentOpponents.remove(bestViewableOppMatching);
			previousOpponents.remove(bestPrevOppMatching);
		}

		for (Car newlyViewableOpponent : currentOpponents)
		{
			opponentData.add(newlyViewableOpponent);
		}

		for (Car notViewableOpponent : previousOpponents)
		{
			notViewableOpponent.setUnseen(true);
		}
	}

	public Map<String, Double> getOnTrackValues() {
		return onTrackValues;
	}



	/* ################################################
	 * ################   Visualization ###############
	 * ################################################ */

	private JSlider[] jss;
	private JLabel[] cValues;
	private String[] keySet;
	private double[] sScales;
	private JButton jb; 
	private boolean doRadicalBreak = false;
	private boolean doSteerLeft = false;

	private void activateValueControl() {
		Set<String> ks = evolvedValues.keySet();
		keySet = new String[ks.size()];
		keySet = ks.toArray(keySet);
		int numKeys = keySet.length;

		jss = new JSlider[numKeys];
		JPanel jps = new JPanel();
		jps.setLayout(new GridLayout(numKeys+1, 3));
		JLabel jls[] = new JLabel[numKeys];
		JFrame jf = new JFrame("");
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setLayout(new BorderLayout());

		JLabel hl = new JLabel("Override control...");
		jb = new JButton("click to break");
		jps.add(hl);
		jps.add(jb);
		jps.add(new JLabel());
		jb.addActionListener(this);

		sScales = new double[numKeys];
		cValues = new JLabel[numKeys];
		for(int i=0; i<numKeys; i++) {
			double keyVal = evolvedValues.get(keySet[i]);
			sScales[i] = keyVal/50;
			double init = 50;
			if(keyVal < .5) {
				sScales[i] = .01;
				init = keyVal / sScales[i];
			}
			jss[i] = new JSlider(0, 100, (int)init);
			jls[i] = new JLabel(keySet[i]+" (scale="+((float)sScales[i])+")");
			jps.add(jls[i]);
			cValues[i] = new JLabel(""+keyVal);
			jps.add(cValues[i]);
			jps.add(jss[i]);
		}
		jf.add(jps, BorderLayout.CENTER);

		// add listeners
		for(int i=0; i<numKeys; i++) {
			jss[i].addChangeListener(this);
		}

		jf.setSize(500, 800);
		jf.setLocation(650, 0);
		jf.setVisible(true);

	}


	public void actionPerformed(ActionEvent arg0) {
		if(arg0.getSource() == jb) {
			if(!doSteerLeft) {
				doSteerLeft = true;
				jb.setText("release break");
			}else{
				doSteerLeft = false;
				jb.setText("click to break");
			}
		}
	}

	public void stateChanged(ChangeEvent arg0) {
		for(int i=0; i<jss.length; i++) {
			if(arg0.getSource() == jss[i]) {
				double keyVal = jss[i].getValue();
				double actualVal = sScales[i] * keyVal;
				cValues[i].setText(""+actualVal);
				cValues[i].repaint();
				evolvedValues.put(keySet[i], actualVal);
			}
		}
	}

	private void activateMonitor() {
		JFrame jf = new JFrame("COBOSTAR Monitor: Driver's View");
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		monitor = new MonitorPanel();
		jf.setLocation(570, 0);
		jf.add(monitor);
		jf.pack();
		jf.setVisible(true);
	}

}