package champ2010client;

public class LeastSquaresInterpolation {
	
	private int order;
	
	public LeastSquaresInterpolation(int order) {
		this.order = order;
	}
	
	private static void product(double[][] m1, double[][] m2, double[][] ret) {
		for(int i=0; i<m1.length; i++) {
			for(int j=0; j<m2[0].length; j++) {
				ret[i][j]=0;
				for(int k=0; k<m1[i].length; k++) ret[i][j] += m1[i][k]*m2[k][j];
			}
		}
	}
	
	private static void product(double[][] m, double[] v, double[] ret) {
		for(int i=0; i<m.length; i++) {
			for(int j=0; j<m[i].length; j++) ret[i] += m[i][j]*v[j];
		}
	}
	
	private static void transpose(double[][] m, double[][] ret) {
		for(int i=0; i<m.length; i++) 
			for(int j=0; j<m[i].length; j++)
				ret[j][i] = m[i][j];
	}
	
	/**
	 * only works for a 3x3 matrix!
	 */
	private static void inverse(double[][] m, double[][] ret) {
		int rows = 3;
		int cols = 3;
		double[][] mAug = new double[5][5];
		put(m, mAug, 5, 5);
		
		double sum1, sum2, det=0;
		for(int j=0; j<cols; j++) {
			sum1 = 1; sum2 = 1;
			for(int i=0; i<rows; i++) {
				sum1*=mAug[i][j+i];
				sum2*=mAug[i][j+2-i];
			}
			det += sum1-sum2;
		}

		for(int i=0; i<rows; i++)
			for(int j=0; j<cols; j++) {
				ret[j][i] = (mAug[i+1][j+1]*mAug[i+2][j+2] -
					mAug[i+1][j+2]*mAug[i+2][j+1])/det;
			}
	}
	
	private static void put(double[][] m, double[][] dest, int rows, int cols) {
		int rowsM = m.length;
		int colsM = m[0].length;
		for(int i=0; i<rows; i++) 
			for(int j=0; j<cols; j++)
				dest[i][j] = m[i%rowsM][j%colsM];
	}
	
	private static void putPoints(double[] x, double[] y, double[][] dest) {
		int rows = x.length;
		int cols = dest[0].length;
		for(int i=0; i<rows; i++)
			for(int j=0; j<cols; j++) {
				dest[i][j]=1;
				for(int k=1; k<=j; k++) dest[i][j]*=x[i];
			}
	}
	
	public static class Result {
		public double maxAngle, maxDistance;
		
		public Result(double maxAngle, double maxDistance) {
			this.maxAngle = maxAngle;
			this.maxDistance = maxDistance;
		}
	}
	
	public Result putPoints(int[] x, double[] y) {
		int rows = x.length;
		double[] xD = new double[rows];
		for(int i=0; i<rows; i++) xD[i] = x[i];
		return putPoints(xD, y);
	}
	
	public Result putPoints(double[] x, double[] y) {
		int cols = order+1;
		int rows = x.length;
		double[][] m = new double[rows][cols];
		putPoints(x, y, m);
		double[][] mT = new double[cols][rows];
		transpose(m, mT);
		double[][] prod = new double[cols][cols];
		product(mT, m, prod);
		double[][] inv = new double[cols][cols];
		inverse(prod, inv);
		double[][] prod1 = new double[cols][rows];
		product(inv, mT, prod1);
		double[] params = new double[order+1];
		product(prod1, y, params);
		return new Result(-params[1]/params[2]/2, 
				params[0]-params[1]*params[1]/params[2]/4);
	}
	
	//for testing purposes
	public static void main(String[] args) {
		LeastSquaresInterpolation interpolation = new LeastSquaresInterpolation(2);
		double[] x = {6, 7, 8, 9, 10};
        double[] y = {40, 70, 160, 80, 50};
        Result res = interpolation.putPoints(x, y);
        System.out.println("maxAngle: "+res.maxAngle);
        System.out.println("maxDistance: "+res.maxDistance);
	}
	
	private static void out(String caption, double[][] m) {
		System.out.println(caption);
		for(int i=0; i<m.length; i++)
			for(int j=0; j<m[i].length; j++)
				System.out.print(m[i][j]+(j<m[i].length-1 ? ", " : "\n"));
	}
	
	private static void out(String caption, double[] v) {
		System.out.println(caption);
		for(int i=0; i<v.length; i++)
			System.out.print(v[i]+(i<v.length-1 ? ", " : "\n"));
	}
}
