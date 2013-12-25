package dataRecording;

import pacman.game.util.*;

/**
 * This class uses the IO class in the PacMan framework to do the actual saving/loading of
 * training data.
 * @author andershh
 *
 */
public class DataSaverLoader {
	public static void SavePacManData(DataTuple data, String FileName)
	{
		IO.saveFile(FileName, data.getSaveString(), true);
	}
	
	public static DataTuple[] LoadPacManData(String FileName)
	{
		String data = IO.loadFile(FileName);
		String[] dataLine = data.split("\n");
		DataTuple[] dataTuples = new DataTuple[dataLine.length];
		
		for(int i = 0; i < dataLine.length; i++)
		{
			dataTuples[i] = new DataTuple(dataLine[i]);
		}
		
		return dataTuples;
	}
}
