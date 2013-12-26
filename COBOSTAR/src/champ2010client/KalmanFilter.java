package champ2010client;

public class KalmanFilter {
	
	private static final boolean ignoreAllThis = false;
	private static final int size = 19;
	//private double[] distance = new double[size], variance = new double[size];
	private double[] distance, variance;
	private double[] x = new double[size], y = new double[size];
	private double[] old = new double[size];
	private final static double stepTime = 0.02;
	private final static double angleDistance = (double)10/360*2*Math.PI;
	private final static double maxValue = 170;
	private static final double noMoveValue = 165;
	private double distanceRaced;
	double[] kalman = new double[size];
	//constants
	private double[] angle = new double[size];
	//private int[] pos = new int[size];
	private double measurementVariance = 100;
	// for debugging purposes
	private int time;
	public static final int maxDebugTime=(int)1E6;
	private boolean doShowOutputs;
	
	private SomeOutputWindow outputWindow;
	
	public KalmanFilter(SomeOutputWindow outputWindow) {
		for(int i=0; i<size; i++) angle[i]=((double)i*10)/360*2*Math.PI;
		this.outputWindow = outputWindow;
		doShowOutputs = (outputWindow!=null);
		//out("angle: ",angle);
	}
	
	private void angleDistanceToLocation() {
		for(int i=0; i<size; i++) {
			x[i]=-Math.cos(angle[i])*distance[i];
			y[i]=Math.sin(angle[i])*distance[i];
		}
	}
	
	private void move(double frontVelocity, double sideVelocity) {
		double frontAdd = frontVelocity/3.6*stepTime;
//		if(time<maxDebugTime) System.out.println("frontAdd: "+frontAdd);
		double sideAdd = sideVelocity/3.6*stepTime;
		for(int i=0; i<size; i++) {
			/*if(y[i]<noMoveValue) */y[i] -= frontAdd;
			x[i] -= sideAdd;
			double x1= x[i], y1=y[i];
			distance[i] = Math.sqrt(x1*x1+y1*y1);
		}
	}
	
	/*private void move(double distanceRaced, double sideVelocity) {
		//System.out.println("distanceRaced: "+distanceRaced);
		double sideAdd = sideVelocity/3.6*stepTime;
		for(int i=0; i<size; i++) {
			if(y[i]<noMoveValue) y[i] -= distanceRaced;
			//x[i] -= sideAdd;
		}
	}*/
	
	private static final double[] clone(double[] values) {
		double[] ret = new double[values.length];
		for(int i=0; i<values.length; i++) ret[i] = values[i];
		return ret;
	}
	
	private void retrieveAngles() {
		for(int i=0; i<size; i++) {
			if(x[i]==0) old[i] = Math.PI/2;
			else{
				old[i] = -Math.atan(y[i]/x[i]);
				if(x[i]>0) old[i]+=Math.PI;
			}
		}
//		if(time<maxDebugTime) out("angles calculated from old distances: ", old);
		double[] helpDistance = new double[size], helpVariance = new double[size];
		double angle2;
		for(int i=0; i<size; i++) { //for each new angle that has to be calculated
			double goal = angle[i];
			for(int j=0; j<size; j++) { //for each old angle
				if(old[j]>=angle[i] || j==size-1) {
					//if(time<maxDebugTime) System.out.println("found old index "+j+" for constant angle index "+i);
					if (j==0 || (j==size-1 && old[j] <= angle[i])) helpDistance[j] = old[j];
					else{
						double factor1 = Math.abs(old[j]-goal)/angleDistance;
						double factor2 = Math.abs(old[j-1]-goal)/angleDistance;
						//if(time<maxDebugTime) System.out.println("factor1: "+factor1+", factor2: "+factor2);
						helpDistance[i] = factor2*distance[j]+factor1*distance[j-1];
						helpVariance[i] = factor2*variance[j]+factor1*variance[j-1];
						break;
					}
				}
			}
		}
		distance = helpDistance;
		variance = helpVariance;
	}
	
	/*private void retrieveAngles() {
		for(int i=0; i<size; i++) {
			if(x[i]==0) old[i] = Math.PI/2;
			else{
				old[i] = -Math.atan(y[i]/x[i]);
				if(x[i]>0) old[i]+=Math.PI;
			}
		}
		if(time<maxDebugTime) out("old locations x: ",x);
		if(time<maxDebugTime) out("old locations y: ",y);
		if(time<maxDebugTime) out("angles calculated from old distances: ", old);
		double[] helpDistance = new double[size], helpVariance = new double[size];
		double angle2;
		for(int i=0; i<size; i++) { //for each new angle that has to be calculated
			double goal = angle[i];
			for(int j=0; j<size; j++) { //for each old angle
				if(old[j]>=angle[i] || j==size-1) {
					//if(time<maxDebugTime) System.out.println("found old index "+j+" for constant angle index "+i);
					if (j==0 || (j==size-1 && old[j] <= angle[i])) helpDistance[j] = old[j];
					else{
						helpDistance[i] = interpolateDistance(angle[i], j-1, j, distance[j-1], distance[j], old);
						break;
					}
				}
			}
		}
		distance = helpDistance;
		variance = helpVariance;
	}*/
	
	/**
	 * 
	 * @param phi interpolationPoint
	 * @param phi1 angle value from old time step smaller than the interpoaltion point
	 * @param phi2 angle value from old time step larger than the interpoaltion point
	 * @param distance1 distance to wall for point phi1
	 * @param distance2 distance to wall for point phi2
	 * @return distance for interpolationPoint phi
	 */
	public double interpolateDistance(double phi, int index1, int index2, double distance1, double distance2, double[] test) {
		double piHalf = Math.PI/2; double accuracy = angleDistance/2;
		if(phi>piHalf-accuracy && phi < piHalf + accuracy) {
			double x1 = Math.abs(x[index1]), x2 = Math.abs(x[index2]);
			double y1= (y[index1]*x2+y[index2]*x1)/(x1+x2);
			return Math.sqrt(x1*x1+y1*y1);
		}
		double tanPhi = Math.tan(phi);
		double x0 = (- x[index2]*y[index1]+x[index1]*y[index2])/(y[index2]-y[index1]+(x[index1]-x[index2])*tanPhi);
		double y0 = -tanPhi * x0;
		/*if(time<maxDebugTime) System.out.println("\nindex1: "+index1+" angle["+index1+"]: "+ test[index1]+" angle["+index2+"]: "+ test[index2]+", phi: "+phi);
		if(time<maxDebugTime) System.out.println("x["+index1+"]: "+x[index1]+", x["+index2+"]: "+x[index2]+", x: "+x0);
		if(time<maxDebugTime) System.out.println("y["+index1+"]: "+y[index1]+", y["+index2+"]: "+y[index2]+", y: "+y0);
		if(time<maxDebugTime) System.out.println("distance: "+Math.sqrt(x0*x0+y0*y0));*/
		return Math.sqrt(x0*x0+y0*y0);
	}
	
	private void addSensorInformation(double[] measurement, int wasOffTrackCounter) {
		if(doShowOutputs) outputWindow.putVariance(variance);
		rotate(measurement, wasOffTrackCounter);
		double uncertainty = 1-1E-1;
		for(int i=0; i<size; i++) variance[i]/=uncertainty;
		for(int i=0; i<size; i++) {
			kalman[i]=variance[i]/(variance[i]+measurementVariance);
			variance[i]=(1-kalman[i])*variance[i];
			double dis = measurement[i];
			if(dis<0) dis=0;
			this.distance[i]=kalman[i]*dis+(1-kalman[i])*this.distance[i];
			if(this.distance[i]>maxValue) this.distance[i] = maxValue;
		}

//		out("variance: ",variance);
//		out("kalman: ",kalman);
//		out("distance: ",distance);
	}
	
	/*private void rotate(double[] measurement) {
		double[] firstDerivative = new double[size-1];
		double[] secondDerivative = new double[size-2];
		double[] firstDerivativeMeasurement = new double[size-1];
		for(int i=0; i<size-1; i++) {
			firstDerivative[i] = distance[i+1]-distance[i];
		}
		for(int i=0; i<size-2; i++) {
			secondDerivative[i] = firstDerivative[i+1]-firstDerivative[i];
		}
		for(int i=0; i<size-1; i++) {
			firstDerivativeMeasurement[i] = measurement[i+1]-measurement[i];
		}
		double deltaPhi = 0;
		for(int i=0; i<size-2; i++) {
			deltaPhi+=(firstDerivative[i]-firstDerivativeMeasurement[i])/secondDerivative[i];
		}
		deltaPhi/=(size-2);
		int shift1 = (int)deltaPhi, shift2=shift1+1;
		double weight1 = deltaPhi-shift1, weight2 = shift2-deltaPhi;
		for(int i=0; i<size; i++) {
			distance[i-]
		}
	}*/
	
	/**
	 * Warning: changes parameter firstDerivative
	 */
	private double getRotationIndex(double[] measurement, double[] firstDerivative) {
		int count=0;
		final double minAbsDerivative = 0.1;
		for(int i=0; i<size-1; i++) {
			firstDerivative[i] = distance[i+1]-distance[i];
		}

		double deltaPhi = 0;
		for(int i=0; i<size-1; i++) {
			double derivative = firstDerivative[i];
			if (!isIn(derivative, -minAbsDerivative, minAbsDerivative)) {
				deltaPhi+=(distance[i]-measurement[i])/derivative;
				count++;
			}
		}
		deltaPhi/=count;
		return deltaPhi;
	}
	
	private double getIn(double value, double min, double max) {
		if(value<min) return min;
		if(value>max) return max;
		return value;
	}
	
	private boolean isIn(double value, double min, double max) {
		if(value<min) return false;
		if(value>max) return false;
		return true;
	}
	
	private int getIn(int value, int min, int max) {
		if(value<min) return min;
		if(value>max) return max;
		return value;
	}
	
	private void rotate(double[] measurement, int wasOffTrackCounter) {
		int maxAbsRotation = 1;
		double[] beforeRotation = clone(distance);
//		out("measurement: ", measurement);
//		outMax("measurement", measurement);
//		out("before rotation: ", distance);
//		outMax("before rotation: ", distance);
		double[] firstDerivative = new double[size-1];;
		double deltaPhi = getRotationIndex(measurement, firstDerivative);
		if(!isIn(deltaPhi, -maxAbsRotation, maxAbsRotation)) {
			if(doShowOutputs) outputWindow.putData1(beforeRotation, distance, measurement, 0, 0, 1, wasOffTrackCounter);
			return;//deltaPhi = 0;
		}
		int shift1 = (int)Math.floor(deltaPhi); 
		int shift2=shift1+1;
//		System.out.println("deltaPhi: "+deltaPhi);
//		System.out.println("shift1: "+shift1+", shift2: "+shift2);
		double weight1 = 1-(deltaPhi-shift1), weight2 = 1-(shift2-deltaPhi);
		weight1 = getIn(weight1, 0, 1);
		weight2 = getIn(weight2, 0, 1);
//		System.out.println("weight1: "+weight1+", weight2: "+weight2);
		for(int i=0; i<size; i++) {
			if(i-shift1>=size) {
				int derivativeIndex = shift1/2+size-1;
				derivativeIndex = getIn(derivativeIndex, 0, size-2);
				distance[i] = distance[size-1] + firstDerivative[derivativeIndex]*(i-deltaPhi-(size-1));
			}else if(i-shift2<0) {
				int derivativeIndex = shift2/2;
				derivativeIndex = getIn(derivativeIndex, 0, size-2);
				distance[i] = distance[0] + firstDerivative[derivativeIndex]*(i-deltaPhi);
			}else {
				distance[i] = distance[i-shift1]*weight1+distance[i-shift2]*weight2;
			}
		}
		if(doShowOutputs)outputWindow.putData1(beforeRotation, distance, measurement, deltaPhi, weight1, weight2, wasOffTrackCounter);
//		System.out.println("shift: "+deltaPhi);
//		out("after rotation", distance);
//		outMax("after rotation", distance);
//		if(time<maxDebugTime) System.out.println("\n");
	}
	
	public void out(String caption, double[] values) {
//		double max=Double.NEGATIVE_INFINITY; int maxIndex=0;
		String s = caption+": ";
		for(int i=0; i<values.length; i++) {
			s+=values[i]+(i<values.length-1 ? ", " : "");
			/*if(values[i]>max) {
				max = values[i];
				maxIndex = i;
			}*/
		}
		if(time<maxDebugTime) {
			System.out.println(s);
//			System.out.println("maximum Index: "+maxIndex+", value: "+max);
		}
	}
	
	public void outMax(String caption, double[] values) {
		double max = Double.MIN_VALUE; int maxIndex=0;
		for(int i=0; i<values.length; i++) if(max<values[i]) {
			max=values[i];
			maxIndex=i;
		}
		if(time<maxDebugTime) System.out.println(caption+" max: "+max+", index: "+maxIndex);
	}
	
	/**
	 * 
	 * @param frontVelocity
	 * @param sideVelocity
	 * @param measurement
	 * @param time for debugging purposes only
	 * @return
	 */
	public double[] getNextEstimation(double frontVelocity, double sideVelocity, double[] measurement, int time, int wasOffTrackCounter) {
		if(ignoreAllThis) return measurement;
		this.time = time;
//		out("\n time "+time+"measurement", measurement);
		if(distance != null && wasOffTrackCounter < 29) {
			angleDistanceToLocation();
//			if(time<maxDebugTime) System.out.println("\nfrontvelocity: "+frontVelocity+"\n");
			move(frontVelocity, sideVelocity);
			this.distanceRaced = distanceRaced;
			retrieveAngles();
			addSensorInformation(measurement, wasOffTrackCounter);
//			outMax("\n distance", distance);
//			out("\n time "+time+"distance", distance);
			double[] test = scale(distance, 1);
//			double[] test = clone(distance);
			return clone(test);
		}
		if(wasOffTrackCounter >=29 ) {
			distance = null;
			setAll(variance, measurementVariance);
		}
		distance = clone(measurement);
		variance = new double[size];
		setAll(variance, measurementVariance);
		double[] test = clone(measurement);
		return clone(test);
	}
	
	public static double[] scale(double[] values, double scale) {
		double[] ret = new double[values.length];
		for(int i=0; i<ret.length; i++)
			ret[i] = values[i]*scale;
		return ret;
	}
	
	public static void setAll(double[] values, double initializationvalue) {
		for(int i=0; i<values.length; i++) values[i] = initializationvalue;
	}
}
