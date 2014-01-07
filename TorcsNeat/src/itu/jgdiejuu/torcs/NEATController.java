package itu.jgdiejuu.torcs;

import com.anji.integration.Activator;
import itu.jgdiejuu.torcs.Controller;

public class NEATController extends Controller{
	
	private static final int MAX_JUMP = 100;
	private static final int TICKS_PER_SAVE = 125; // Approx. 5 seconds.
	
	// distance
	private Activator activator;
	private double dist = 0.0;
	private double lastLaps = 0.0;
	private int laps = 0;
	
    // change in position in the last TICKS_PER_SAVE
    private double lastDiff = 10.0;
    private double lastDist = 0.0;
    private int lastTick = 0;
    private int tick = 0;

	// position and overtakes
	private int curPos = 1;
	private int ownTakes = 0;
	private int taken = 0;
	
	private boolean manualGear;

	public NEATController(Activator acti, boolean manualGear){
		this.activator = acti;
		this.manualGear = manualGear;
	}
	
	public void reset() {
		//System.out.println("Restarting the race!");
		
	}

	@Override
	public Action control(SensorModel sensors) {
        double dist = sensors.getDistanceFromStartLine();
        updateDiff(dist);
        storeDistance(dist);
        storePosition(sensors.getRacePosition());
		
		Action result = convertOutput(activator.next(covertInput(sensors)));
		if(!manualGear){
			result.clutch = 0;//clamp(output[2],0,1);<--old clutch
			result.gear = automaticGear(sensors);
		}
		return result;
	}

	//based on Anders' code - rewritten for 
	private int automaticGear(SensorModel sensormodel){
		
		int gear = sensormodel.getGear();
			
		switch(gear){
			case 6: if(sensormodel.getRPM() < 2000){gear = 5;} break;
			case 5: if(sensormodel.getRPM() > 9000){gear = 6;}else if(sensormodel.getRPM() < 2000){gear = 4;} break;
			case 4:	if(sensormodel.getRPM() > 9000){gear = 5;}else if(sensormodel.getRPM() < 2000){gear = 3;} break;
			case 3: if(sensormodel.getRPM() > 9000){gear = 4;}else if(sensormodel.getRPM() < 2000){gear = 2;} break;
			case 2: if(sensormodel.getRPM() > 9000){gear = 3;}else if(sensormodel.getRPM() < 2000){gear = 1;} break;
			case 1: if(sensormodel.getRPM() > 9000){gear = 2;} break;
			case 0: gear = 1; break;
			case -1:  gear = 1; break;
		}
		return gear;				
	}
	
    private void updateDiff(double dist) {
        if(tick == lastTick + TICKS_PER_SAVE){
                lastTick = tick;
                lastDiff = Math.abs(dist - lastDist);
                lastDist = dist;
        }
        tick++;
    }

	private double[] covertInput(SensorModel sensors) {
		double[] result = new double[manualGear ? 26 : 24];
		//bias
		result[0] = 1.0;
		
		result[1] = normalizeAngle(sensors.getAngleToTrackAxis());
		// not used curLapTime
		// not used damage
		// not used distFromStart
		// not used distRaced
		// not used focus
		// not used fuel
		//not used lastLapTime
		
		double[] opp = normalizeOpponents(sensors.getOpponentSensors());
		result[2] = opp[0];
		result[3] = opp[1];
		result[4] = opp[2];
		result[5] = opp[3];
		result[6] = opp[4];
		result[7] = opp[5];
		result[8] = opp[6];
		result[9] = opp[7];
		// not used racePos	
		result[10] = normalizeSpeed(sensors.getSpeed()); // speedX
		result[11] = normalizeSpeed(sensors.getLateralSpeed()); // speedY
		result[12] = normalizeSpeed(sensors.getZSpeed()); // speedZ
		double[] track = normalizeTrack(sensors.getTrackEdgeSensors());
		result[13] = track[0];
		result[14] = track[1];
		result[15] = track[2];
		result[16] = track[3];
		result[17] = track[4];
		result[18] = track[5];
		result[19] = track[6];
		result[20] = track[7];
		result[21] = track[8];
		result[22] = track[9];
		result[23] = normalizeTrackPos(sensors.getTrackPosition());
		// not used wheelSpinVel
		// not used z
		
		if(manualGear){
			result[24] = normalizedGear(sensors.getGear());
			result[25] = normalizeRPM(sensors.getRPM());
		}
		
		return result;
	}

	private double normalizeTrackPos(double pos) {
		return (pos + 1.0)/2.0;
	}

	private double[] normalizeTrack(double[] track) {
		double[] result = new double[10];
		for(int i = 0; i < result.length; i++){
			result[i] = track[i*2]/200.0;
		}
		return result;
	}

	private double normalizeSpeed(double speed) {
		return (speed+200)/400.0;
	}

	private double normalizeRPM(double rpm) {
		return (rpm-2000)/5000.0;
	}

	private double[] normalizeOpponents(double[] opp) {
		double[] result = new double[8];
		for(int i = 0; i < result.length; i++){
			result[i] = opp[i*4]/200.0;
		}
		return result;
	}

	private double normalizedGear(int gear) {
		return (gear+1)/8.0;
	}

	private double normalizeAngle(double angle) {
		return (angle+Math.PI)/(2*Math.PI);
	}

	private Action convertOutput(double[] output) {
		Action result = new Action();
		result.accelerate = clamp(output[0],0,1);
		result.brake = clamp(output[1],0,1);
		result.steering = normalizeSteering(output[2]);
		
		if(manualGear){
			//result.clutch = clamp(output[3],0,1);
			result.gear = getGear(output[3]);
		}
		
		// not used focus
		// not used meta
		return result;
	}
	
	private double normalizeSteering(double steer) {
		return clamp(steer*2.0-1.0,-1,1);
	}

	private int getGear(double gear) {
		int result = (int)(gear*9.0-1.0);
		return result == 8 ? 7 : result;
	}

	private double clamp(double value, double min, double max){
		return Math.max(min, Math.min(max, value));
	}
	
	private void storeDistance(double newFit) {
		if(newFit > dist && dist + MAX_JUMP > newFit){ // legal progress
			dist = newFit;
		}else if(newFit < MAX_JUMP && dist > 5*MAX_JUMP){ // new lap
			laps++;
			lastLaps += dist;
			dist = newFit;
		}
	}
	
	private void storePosition(int racePosition) {
		if(racePosition > curPos){ // overtaken someone
			ownTakes++;
		}else if(racePosition < curPos){ // someone overtook
			taken++;
		}
	
		curPos = racePosition;
	}
	
	public int getLaps(){
		return laps;
	}
	
	// Gets the last known position. 1 is best.
	public int getCurrentPosition(){
		return curPos;
	}
	
	// Times the controller has overtaken others.
	public int getNumberOvertaken(){
		return taken;
	}
	
	// Times the controller HAS BEEN overtaken by others.
	public int getNumberOvertakes(){
		return ownTakes;
	}

	// The farthest the controller has been from the start line + laps
	public int getMaxDistance() {
		System.out.println(">> Get fitness: "+(dist+lastLaps));
		return (int)(dist + lastLaps);
	}
	
	// Meters moved last 5 seconds.
	public double getDeltaFive(){
		return lastDiff;
	}

	@Override
	public void shutdown() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public String toString(){
		return "Dist: "+this.getMaxDistance()
				+" Laps: "+this.getLaps()
				+" Taken: "+this.getNumberOvertaken()
				+" Overtakes: "+this.getNumberOvertakes()
				+" Pos: "+this.getCurrentPosition();
	}
}
