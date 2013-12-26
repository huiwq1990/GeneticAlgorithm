package champ2010client;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridLayout;

import javax.swing.JFrame;
import javax.swing.JLabel;

public class SomeOutputWindow extends JFrame {
	
	private JLabel distancesBeforeRotation, distancesAfterRotation,
	measurement, shift, weight1, weight2, wasOffTrackCounter,
	variance, minOpponentDistance, doOpponentAvoidance,
	maxAngleIndices, maxDistances, interpolatedAngle, interpolatedDistance,
	targetSpeed, targetSpeedBeforeAvoidance;
	
	public SomeOutputWindow() {
		super("Output");
		Container cp = getContentPane();
		distancesBeforeRotation = new JLabel("distance before Rotation: ");
		distancesAfterRotation = new JLabel("distance after Rotation: ");
		measurement = new JLabel("measurement: ");
		shift = new JLabel("shift");
		weight1 = new JLabel("weight1");
		weight2 = new JLabel("weight2");
		wasOffTrackCounter = new JLabel("wasOffTrackCounter");
		maxAngleIndices = new JLabel("maxAngleIndex: ");
		maxDistances = new JLabel("maxDistance: ");
		interpolatedAngle = new JLabel("interpolatedAngle: ");
		interpolatedDistance = new JLabel("interpolatedDistance: ");
		variance = new JLabel("Variance: ");
		minOpponentDistance = new JLabel("minOpponentDistance: ");
		doOpponentAvoidance = new JLabel("opponentAvoidance inactive");
		targetSpeed = new JLabel("targetSpeed");
		targetSpeedBeforeAvoidance = new JLabel("targetSpeed before Avoidance: ");
		setSize(new Dimension(700, 200));
		cp.setLayout(new GridLayout(7,1));
		cp.add(minOpponentDistance);
		cp.add(doOpponentAvoidance);
		cp.add(targetSpeedBeforeAvoidance);
		cp.add(targetSpeed);
		/*cp.add(measurement);
		cp.add(variance);
		cp.add(distancesBeforeRotation);
		cp.add(distancesAfterRotation);
		cp.add(shift);
		cp.add(weight1);
		cp.add(weight2);
		cp.add(wasOffTrackCounter);
		cp.add(maxAngleIndices);
		cp.add(maxDistances);
		cp.add(interpolatedAngle);
		cp.add(interpolatedDistance);*/
		setLocation(0, 530);
		setVisible(true);
	}
	
	public static void main(String[] args) {
		new SomeOutputWindow();
	}
	
	private String arrayToString(double[] values) {
		if (values == null) return "";
		String ret = "";
		for(int i=0; i<values.length; i++)
			ret+=(int)values[i]+ (i<values.length-1 ? ", \t" : "");
		return ret;
		
	}
	
	private String arrayToString(int[] values) {
		if (values == null) return "";
		String ret = "";
		for(int i=0; i<values.length; i++)
			ret+=(int)values[i]+ (i<values.length-1 ? ", \t" : "");
		return ret;
		
	}
	
	public void putData1(double[] distancesBeforeRotation, double[] distancesAfterRotation, double[] measurement, double shift, double weight1, double weight2, int wasOffTrackCounter) {
		this.distancesBeforeRotation.setText("distance before Rotation:	"+arrayToString(distancesBeforeRotation));
		this.distancesAfterRotation.setText	("distance after Rotation:	"+arrayToString(distancesAfterRotation));
		this.measurement.setText			("measurement:				"+arrayToString(measurement));
		this.shift.setText("shift: "+shift);
		this.weight1.setText("weight: "+weight1);
		this.weight2.setText("weight2: "+weight2);
		this.wasOffTrackCounter.setText("wasOffTrackCounter: "+wasOffTrackCounter);
	}
	
	public void putData2(int[] maxAngleIndices, double[] maxDistances, double interpolatedAngle, double interpolatedDistance) {
		this.maxAngleIndices.setText("maxAngleIndices: "+arrayToString(maxAngleIndices));
		this.maxDistances.setText("maxDistances: "+arrayToString(maxDistances));
		this.interpolatedAngle.setText("interpolatedAngle: "+interpolatedAngle);
		this.interpolatedDistance.setText("interpolatedDistance: "+interpolatedDistance);
	}
	
	public void putVariance(double[] variance/*, double[] measurementVariance*/) {
		this.variance.setText("Variance: "+arrayToString(variance));
//		this.measurementVariance.setText("measurementVar: "+arrayToString(measurementVariance));
	}
	
	public void putOpponentDistance(double minOpponentDistance) {
		this.minOpponentDistance.setText("minOpponentDistance: "+minOpponentDistance);
	}
	
	public void putOpponentAvoidance(boolean doOpponentAvoidance) {
		this.doOpponentAvoidance.setText("opponentAvoidance "+(doOpponentAvoidance ? "active" : "inactive"));
	}
	
	public void putTargetSpeed(double targetSpeedBeforeAvoidance, double targetSpeedAfterAvoidance) {
		this.targetSpeed.setText("targetSpeed: "+targetSpeedAfterAvoidance);
		this.targetSpeedBeforeAvoidance.setText("targetSpeed before Avoidance: "+targetSpeedBeforeAvoidance);
	}
}
