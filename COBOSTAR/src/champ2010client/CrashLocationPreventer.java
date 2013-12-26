package champ2010client;

import java.util.Vector;

public class CrashLocationPreventer {

	private COBOSTAR racer; 

	/**
	 * Need those to know how the racer generally behaves.
	 * Deduced those by test done by hand. 
	 * Dirt track or normal track does not seeem to make much of a difference 
	 * with respect to these values... 
	 */
	private double[] racerAccelMeasurePoints = {0, 50, 200, 300, 1000};
	private double[] racerAccelVelIncrase = {1, 1, 0,25, 0.02, 0};
	
	private double[] racerBreakMeasurePoints = {0, 10, 50, 100, 300, 1000};
	private double[] racerBreakVelDecrase = {0, 1, 1.75, 2, 3.3, 5};

	private double[] racerNoAccNoBreakPoints = {0, 10, 100, 300, 1000};
	private double[] racerNoAccNoBreakVelDecrease = {0, 0.15, 0.2, 0.35, 0.5};

	private static double getInterpolatedChangeVel(double vel, 
			double[] measurePoints, double[] velDecrease) 
	{
		int i;
		for(i=1; i<measurePoints.length-1; i++) {
			if(measurePoints[i] > vel)
				break;
		}
		return velDecrease[i-1] + 
			(vel-measurePoints[i-1])*(velDecrease[i] - velDecrease[i-1]) /
				(measurePoints[i] - measurePoints[i-1]);
	}
	
	
	/**
	 * Size of recent history window data.
	 */
	private final int historyLength = 1000;

	/**
	 * Window within which one danger zone is monitored.
	 * Note that this length should be e.g. historyLength/2 in order to remember what happened
	 * also before the crash point.
	 */
	private final int detectingDangerZoneWindow = historyLength/2;

	
	// monitoring the last steps using an iterator.
	private int iterator;
	

	/**
	 * Array to store history data.
	 */
	private HistoryProperties[] history;
	
	
	/**
	 * Vector of previously occurred crash locations.
	 */
	private Vector<CrashProperties> crashPositions = new Vector<CrashProperties>();

	/**
	 * Flag that specified if currently inside dangerous (previous crash) zone 
	 * And thus if car behavior (speed) is currently controlled by this CrashLocationPreventer.
	 */
	private boolean insideDangerZone = false;

	/**
	 * The properties of the danger zone the racer is currently in.
	 */
	private CrashProperties currentIDZProperties = null;
	
	/**
	 * If strong lateral speed but still checking if action is required...
	 */
	private boolean detectingDangerZone = false;

	private int ddzDetectionCounter = 0;
	private int ddzOccurrencePoint = 0;
	private double ddzMinOpponentDistance = 0;

	private double ddzWorstOffTrack = 0; 
	private double ddzWorstOrientation = 0;
	private double ddzDamage = 0;
	
	/**
	 * Constructor.
	 * 
	 * @param racer The racer that is monitored
	 */
	public CrashLocationPreventer(COBOSTAR racer)
	{
		this.racer = racer;
		
		history = new HistoryProperties[historyLength];
		
		for(int i=0; i<historyLength; i++)
			history[i] = new HistoryProperties();
	}

	/**
	 * Method that is called each iteration from the racer.
	 * 
	 * @param sensors The sensor model currently perceived
	 * @param targetSpeed The target speed that is calculated by the racer's strategy.
	 */
	public void notifyCrashLocator(SensorModel sensors, double targetSpeed, 
			double currentAccel, double currentBreak)
	{
		// first remember what is currently happening... 
		history[iterator%historyLength].resetProperties(sensors, targetSpeed, currentAccel, currentBreak);
		
		if(!detectingDangerZone) {
			if(Math.abs(history[iterator%historyLength].lateralSpeed) > 30.) {
				detectingDangerZone = true;
				ddzOccurrencePoint = iterator;
				ddzMinOpponentDistance = racer.getClosestOpponentDistance();
				ddzWorstOffTrack = 0;
				ddzWorstOrientation = 0;
				ddzDamage = sensors.getDamage();
			}
			ddzDetectionCounter = 0;
		}else if(detectingDangerZone) {
			double orientation = sensors.getAngleToTrackAxis();
			double onOffTrack = sensors.getTrackPosition();
			ddzDetectionCounter++;
			
			double help = Math.abs(onOffTrack);
			if(help > ddzWorstOffTrack)
				ddzWorstOffTrack = help; 
			
			help = Math.abs(orientation);
			if(help > ddzWorstOrientation)
				ddzWorstOrientation = help;

			if(ddzDetectionCounter >= detectingDangerZoneWindow) {
				// Done monitoring danger zone... let's see how bad it was
				detectingDangerZone = false;
				ddzDetectionCounter = 0;
				ddzDamage = sensors.getDamage()-ddzDamage;

				if(ddzWorstOffTrack > 1.2 || ddzWorstOrientation > 0.5 * Math.PI
						|| ddzDamage > 800)
					addCrashLocation();
			}
		}
		
		iterator++;
	}

	/**
	 * Triggered when a severe crash has occurred.
	 */
	private void addCrashLocation()
	{
		// First - check all previously stored crash locations in order to prevent multiple storing 
		// of the same location
		for(int i=0; i<crashPositions.size(); i++) {
			CrashProperties crashProps = crashPositions.get(i);
			if (crashProps.startLocation - 50 < 
						history[ddzOccurrencePoint%historyLength].relTrackLocation &&
					crashProps.endLocation + 100 >
						history[ddzOccurrencePoint%historyLength].relTrackLocation) {
				
				// location was in fact already stored.
				if(crashProps.endLocTotalDistanceRaced + 2000 < 
							history[ddzOccurrencePoint%historyLength].distanceRaced) {
					// already stored this location but now in the NEXT round
					// move crash point to the front if it occurred even earlier this time.
//					if(history[ddzOccurrencePoint%historyLength].relTrackLocation < 
//							crashProps.endLocation) {
//						crashProps.startLocation -= (crashProps.endLocation - 
//								history[ddzOccurrencePoint%historyLength].relTrackLocation);
//						crashProps.endLocation = history[ddzOccurrencePoint%historyLength].relTrackLocation;
//					}
					
					double severity = getCurrentCrashSeverity();
					
					if(severity > .1) {
						double newSig = crashProps.significance + severity;
						crashProps.changeSignificance(newSig);					
						// update total distance raced to prevent multiple updates in one round
						crashProps.endLocTotalDistanceRaced = 
							history[ddzOccurrencePoint%historyLength].distanceRaced;

						if(COBOSTAR.doSysouts) System.out.println("Updated Crash Location: "+crashProps);
					}else{
						if(COBOSTAR.doSysouts) System.out.println("NO crash location update - non-significant crash: sig="+severity);
					}
				}else{
					if(COBOSTAR.doSysouts) System.out.println("In fact trying to add crash point despite previous one!?");
				}
				// updated or still within previous crash point - not a new crash point!
				return;
			}
		}
		// Location was not previously stored. Thus, remember crash location.
		if(COBOSTAR.doSysouts) System.out.println("Remembering crash location: "+
				history[ddzOccurrencePoint%historyLength]);

		// Now handling what should be done better in the next round.
		if(iterator < historyLength)
			return; // crash occurrence occurred really early - don't handle this weired case.
		
		HistoryProperties hp1 = history[ddzOccurrencePoint%historyLength];
		HistoryProperties hp2;
		
		// First, find high positive speed and low lateral speed before crash point.
		int speedPoint = ddzOccurrencePoint;
		double ls = Math.abs(hp1.lateralSpeed);
		double speed = hp1.actualSpeed;
		for(int i=1; i<0.1 * historyLength; i+=2) {
			hp1 = history[(ddzOccurrencePoint-i)%historyLength];
			hp2 = history[(ddzOccurrencePoint-i-1)%historyLength];
			if(speed > 0 && 
					ls < Math.abs(hp1.lateralSpeed) && 
					ls < Math.abs(hp2.lateralSpeed) && 
					speed > hp1.actualSpeed && 
					speed > hp2.actualSpeed) {
				// speed greater than zero
				// lateral speed is not anymore further decreasing (backwards in time!) AND 
				// speed of car is not further increasing (backwards in time!).
				speedPoint = ddzOccurrencePoint - (i-1);
				break;
			}
			ls = (Math.abs(hp1.lateralSpeed) + Math.abs(hp2.lateralSpeed))/2;
			speed = (hp1.actualSpeed + hp2.actualSpeed)/2;
		}

		if(COBOSTAR.doSysouts) System.out.println("Fastest speed point at: "+speedPoint+" " +
				"with speed:"+speed+" and lateralSpeed:"+ls);
				
		double severity = getCurrentCrashSeverity();
		
		if(COBOSTAR.doSysouts) System.out.println("Severity:"+severity+ " due to worstOrienation:"+
				ddzWorstOrientation+ " and/or worstOffTrack:"+ddzWorstOffTrack+
				" and/or ddzDamage:"+ddzDamage);
		if(severity <= 0.1) {
			if(COBOSTAR.doSysouts) System.out.println("Crash is insignificant - am not remembering it!");
			return;
		}
		
		// Thus, determine speed adjustment value
		double necessarySpeedDecrease = 0.5 * severity * speed;
		double decreasedSpeed = speed - necessarySpeedDecrease;
		if(COBOSTAR.doSysouts) System.out.println("Speed at critical point: "+speed+" " +
				"versus decreased speed: "+decreasedSpeed);

		// creating crash property location point... 
		CrashProperties crashProps = new CrashProperties();
		crashProps.endLocation = history[speedPoint%historyLength].relTrackLocation;
		crashProps.endLocTotalDistanceRaced = history[speedPoint%historyLength].distanceRaced;
		crashProps.significance = severity;
		
		// now move further backwards to adjust that car behavior....
		double diffVel;
		double enforcedChangeInVel;

		int i = ddzOccurrencePoint - speedPoint + 1;
		if(COBOSTAR.doSysouts) System.out.println("Starting to count from: "+i+" occp:"+ddzOccurrencePoint+
				" sp:"+speedPoint);
		
		for( ; 0<= ddzOccurrencePoint-i && i<historyLength-1; i++) 
		{	
			diffVel = history[(ddzOccurrencePoint-i)%historyLength].actualSpeed - 
				history[(ddzOccurrencePoint-i+1)%historyLength].actualSpeed;
			
			// now do the behavioral adjustments... and store them appropriately.!
			enforcedChangeInVel = 0.5 * CrashLocationPreventer.getInterpolatedChangeVel(
					decreasedSpeed, this.racerBreakMeasurePoints, this.racerBreakVelDecrase);
			// add occurred change to deduce maximal change possible.
			enforcedChangeInVel += diffVel;

			necessarySpeedDecrease -= enforcedChangeInVel;

			
//			if(COBOSTAR.doSysouts) System.out.println("Necessary speed decrase: "+necessarySpeedDecrease+
//					" eciv:"+enforcedChangeInVel+" diffVel: "+diffVel);
			if(necessarySpeedDecrease < 0) {
				// done with the full decreasing capability... 
				break;
			}
			
			crashProps.addCrashMonitorPoint(
					history[(ddzOccurrencePoint-i)%historyLength].actualSpeed, 
					history[(ddzOccurrencePoint-i)%historyLength].actualSpeed-necessarySpeedDecrease,
					history[(ddzOccurrencePoint-i)%historyLength].relTrackLocation);
		}
		crashProps.startLocation = history[(ddzOccurrencePoint-i)%historyLength].relTrackLocation;

		if(crashProps.startLocation > crashProps.endLocation) {
			if(crashProps.startLocation > .8 * COBOSTAR.trackLength && 
					crashProps.endLocation < .2 * COBOSTAR.trackLength) {
				crashProps.isBetweenStart = true;
			}else{
				if(COBOSTAR.doSysouts) System.out.println("Start-End Location inconsistency: Start:"+
						crashProps.startLocation+" End:"+crashProps.endLocation+" within a track length of "+COBOSTAR.trackLength);
				return;
			}
		}
		
		// adding the new crash location
		crashPositions.add(crashProps);
		
		if(COBOSTAR.doSysouts) System.out.println("New crash position: "+crashProps);
		//if(COBOSTAR.doSysouts) System.out.println("Full crash properties: "+crashProps.getMonitorString());
	}

	private double getCurrentCrashSeverity()
	{
		// Determine severity of the crash
		// Maximally severity 1 - scaled with off-track and orientation issues.
		double severity = Math.min( 1., 
				Math.max( ( ddzDamage-400 ) / 500., 
				Math.min( Math.sqrt(Math.max(0, ddzWorstOrientation-Math.PI/3)), 
						Math.sqrt(Math.max(0., ddzWorstOffTrack-1.1) ) ) ) );
		// strongly decrease significance in the presence of opponents.
		if(ddzMinOpponentDistance < 15)
			severity *= (ddzMinOpponentDistance / 15.)
				*(ddzMinOpponentDistance / 15.)
				*(ddzMinOpponentDistance / 15.);
		return severity;
	}
	
	/**
	 * Called each iteration to check if inside a dangerous zone (defined by previous crash locations).
	 * 
	 * @param currentLocation The current distance from the start of the track (in this lap).
	 */
	public boolean checkCrashLocations(double currentLocation) 
	{
		if(insideDangerZone == true) {
			if(currentIDZProperties==null) {
				if(COBOSTAR.doSysouts) System.out.println("Danger zone should be set - we are inside!?");
				insideDangerZone = false;
				return false;
			}
			insideDangerZone = currentIDZProperties.updateIterationLocator(currentLocation);
			
			if(insideDangerZone == false) {
				if(COBOSTAR.doSysouts) System.out.println("Left danger zone..."+currentLocation);
			}
			return insideDangerZone;
		}else{		
			// check if this area is covered by a stored crash location
			for(int i=0; i<crashPositions.size(); i++) {
				CrashProperties crashProps = crashPositions.get(i);
				// area is 100Meters before the crash location and 10 after (dependent on impact factor loc[1])
				if( (crashProps.startLocation < currentLocation && 
						crashProps.endLocation > currentLocation) // inside start-end
						|| (crashProps.isBetweenStart &&
								((crashProps.startLocation < currentLocation && 
									crashProps.endLocation < currentLocation) ||
								(crashProps.startLocation > currentLocation && 
										crashProps.endLocation > currentLocation) ) ) // between start line crash point...
							) {
					// entering danger zone 
					currentIDZProperties = crashProps;
					boolean success = currentIDZProperties.activateLocationMonitoring(currentLocation);
					if(success) {
						if(COBOSTAR.doSysouts) System.out.println("Entering danger zone "+currentIDZProperties);
						insideDangerZone = true;
						return true;
					}
				}
			}
		}
		return false;
	}

	public double getCrashPreventionTargetSpeed()
	{
		if(insideDangerZone)
			return currentIDZProperties.ccm[currentIDZProperties.curIteration].targetVel;

		return -1;
	}
	
	//	double location;
	//	double speedReductionFactor;
	//	double totalDistanceRaced;
	//	double absolutValue;
	//	double distanceToOpponent;

	public int getNumCrashPositions()
	{
		return crashPositions.size();
	}

	
	/**
	 * ######################################################################
	 * ########  sub.class HistoryProperties... 
	 * ######################################################################
	 */

	/**
	 * Class to monitor what happened recently in order to adjust strategy effectively in the next round.
	 */
	private class HistoryProperties {
		/**
		 * Track location from starting line
		 */
		double relTrackLocation = 0;
		/**
		 * Distance raced in total.
		 */
		double distanceRaced = 0;
		double actualSpeed = 0;
		double targetSpeed = 0;
		double actualAccel = 0;
		double actualBreak = 0;
		double lateralSpeed = 0;
		
		private HistoryProperties() {
		}
		
		private HistoryProperties(double trackL, double disRaced, double actualS, double targetS, 
				double actualA, double actualB, double lateralS) {
			resetProperties(trackL, disRaced, actualS, targetS, actualA, actualB, lateralS);
		}
		
		void resetProperties(double relTrackL, double disRaced, double actualS, double targetS, 
				double actualA, double actualB, double lateralS) {
			relTrackLocation = relTrackL;
			distanceRaced = disRaced;
			actualSpeed = actualS;
			targetSpeed = targetS;
			actualAccel = actualA;
			actualBreak = actualB;
			lateralSpeed = lateralS;
		}	
		
		void resetProperties(SensorModel sensors, double targetS, 
				double currentAccel, double currentBreak) {
			relTrackLocation = sensors.getDistanceFromStartLine();
			distanceRaced = sensors.getDistanceRaced();
			actualSpeed = sensors.getSpeed();
			targetSpeed = targetS;
			actualAccel = currentAccel;
			actualBreak = currentBreak;
			lateralSpeed = sensors.getLateralSpeed();
		}
		
		public String toString()
		{
			return "loc:"+relTrackLocation+" disR:"+distanceRaced+" speed:"+actualSpeed+" targetS:"+targetSpeed+" " +
					" acceler:"+actualAccel+" break:"+actualBreak+" lateralS:"+lateralSpeed;
			
		}
	}

	/**
	 * ######################################################################
	 * ########  sub.class CrashProperties... 
	 * ######################################################################
	 */

	/**
	 * Subclass that handles crash point history data in order to effectively
	 * adjust the strategy in the next round.
	 */
	private class CrashProperties{
		/**
		 * The location where crash control should kick in.
		 */
		double startLocation;

		/**
		 * The end point where crash control should stop.
		 */
		double endLocation; 

		/**
		 * The total distance raced when last this was updated.
		 */
		double endLocTotalDistanceRaced;

		/**
		 * Current considered significance of the crash.
		 */
		double significance;

		/**
		 * The number of iterations stored for the control part.
		 */
		int numIterations; 
		
		/**
		 * The iteration this activated crash properties
		 * within which the racer is currently located.
		 */
		int curIteration = 0;

		/**
		 * Flag that states if these crash properties span over the start location.
		 */
		boolean isBetweenStart = false;
		
		/**
		 * Variables for the crash control monitor. 
		 */
		CrashControlMonitor[] ccm; 

		private CrashProperties()
		{
			ccm = new CrashControlMonitor[historyLength];
			numIterations = 0;
			significance = 0;
			startLocation = 0;
			endLocation = 0;
			isBetweenStart = false;
		}
		
		/**
		 * Sets the curIteration value to the iteration that corresponds to the current racer's location.
		 * 
		 * @param curRelLoc The current relative location of the racer.
		 * @return true if curIteration was successfully reset. 
		 */
		private boolean updateIterationLocator(double curRelLoc)
		{
			while(curIteration >= 0 && curIteration < numIterations && 
					( ccm[curIteration].relTrackLoc < curRelLoc ||
					( isBetweenStart == true && 
							ccm[curIteration].relTrackLoc > curRelLoc && 
							ccm[curIteration].relTrackLoc > 0.9 * COBOSTAR.trackLength &&
							curRelLoc < 0.1 * COBOSTAR.trackLength ) ) )
				curIteration--;
			if(curIteration < 0 || curIteration >= numIterations)
				return false;
			
			return true;
		}
		
		private boolean activateLocationMonitoring(double curRelLoc) 
		{
			curIteration = numIterations - 1;
			return updateIterationLocator(curRelLoc);
		}
		
		private void addCrashMonitorPoint(double originalVelocity, double desiredVelocity, double relTrackLoc)
		{
			ccm[numIterations] = new CrashControlMonitor(originalVelocity, desiredVelocity, relTrackLoc);
			numIterations++;
		}
		
		private void changeSignificance(double newSignificance) 
		{
			if(significance == 0 || numIterations ==0) {
				if(COBOSTAR.doSysouts) System.out.println("Significance "+significance+
						" numIterations="+numIterations+" ZERO... : " +
								"Thus, cannot change to new Significance: "+newSignificance);
				return;
			}
			
			if(COBOSTAR.doSysouts) System.out.println("Changing Significances of this loc: "+this);
			double sigChangeFactor = newSignificance / significance;

			CrashControlMonitor[] ccmOld = ccm;
			
			int newLength = (int)((double)numIterations * sigChangeFactor);
			ccm = new CrashControlMonitor[historyLength];
			double copyIt = 0;
			int copyPos = 0;
			for(int i=0; i<newLength; i++) {
				if(copyIt < i) {
					copyIt += sigChangeFactor;
					copyPos++;
					if(copyPos >= numIterations)
						copyPos = numIterations-1;
				}
				if(!isBetweenStart) {
					ccm[i] = new CrashControlMonitor(ccmOld[copyPos].originalVel,
						ccmOld[copyPos].originalVel - 
							sigChangeFactor * (ccmOld[copyPos].originalVel - ccmOld[copyPos].targetVel), 
								ccmOld[0].relTrackLoc - 
								sigChangeFactor * (ccmOld[0].relTrackLoc - ccmOld[copyPos].relTrackLoc) );
				}else{
					ccm[i] = new CrashControlMonitor(ccmOld[copyPos].originalVel,
							ccmOld[copyPos].originalVel - 
								sigChangeFactor * (ccmOld[copyPos].originalVel - ccmOld[copyPos].targetVel), 
									ccmOld[copyPos].relTrackLoc );
					
				}
				//if(COBOSTAR.doSysouts) System.out.println(""+i+" vs."+copyPos+" "+ccmOld[copyPos]+ "new: "+ccm[i]);
			}
			significance = newSignificance;
			numIterations = newLength;
			startLocation = ccm[numIterations-1].relTrackLoc;
			curIteration = 0;
		}
		
		public String toString()
		{
			return "startLoc:"+startLocation+" endLoc:"+endLocation+" significance:"+significance+
			" numIterations:"+numIterations;
		}

		public String getMonitorString()
		{
			StringBuffer sb = new StringBuffer();
			for(int i=0; i<numIterations; i++) {
				sb.append("(loc:"+((float)ccm[i].relTrackLoc)+
						" Vels:"+((float)ccm[i].originalVel)+".vs."+((float)ccm[i].targetVel)+")");
			}
			return ""+numIterations+": "+sb.toString();
		}
		
		private class CrashControlMonitor{
			private double originalVel = 0;
			private double targetVel = 0;
			private double relTrackLoc = 0;

			public CrashControlMonitor(double ov, double tv, double rtl) {
				originalVel = ov;
				targetVel = tv;
				if(tv < 10.)
					targetVel = 10.;
				relTrackLoc = rtl;
			}
			
			public String toString()
			{
				return ""+((float)originalVel)+" "+((float)targetVel)+" "+((float)relTrackLoc);
			}
		}
	}

}

