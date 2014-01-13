package run.mario;

import java.io.IOException;

import com.anji.util.Reset;

public class RestFile {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String[] parms = {"mario.properties"};
		try {
			Reset.main(parms);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
