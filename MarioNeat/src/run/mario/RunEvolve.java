package run.mario;

import com.anji.neat.Evolver;

public class RunEvolve {

	public static void main(String[] args)   {
		// TODO Auto-generated method stub
		String[] parms = {"mario.properties"};
		try {
			Evolver.main(parms);
		} catch (Throwable e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
