package PIDStats;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.Date;

public class GatherStatistics {
	private PrintWriter writer;
	private Date time;
	private int count = 0;
	public enum PID_TYPE {TURN, LEFT_ENCODER, RIGHT_ENCODER, ENCODER}

	public GatherStatistics(PID_TYPE pType) throws IOException {
		count++;
		time = new Date();
		File f;
		if(pType == PID_TYPE.TURN)	{
			f = new File("//home//lvuser//deploy//TurnPIDStats" + time.toString() +".txt");
		} else if(pType == PID_TYPE.LEFT_ENCODER){
			f = new File("//home//lvuser//deploy//LeftEncoderPIDStats" + time.toString() +".txt");
		} else if(pType == PID_TYPE.RIGHT_ENCODER){
			f = new File("//home//lvuser//deploy//RightEncoderPIDStats" + time.toString() +".txt");
		} else{
			f = new File("//home//lvuser//deploy//EncoderPIDStats" + time.toString() +".txt");
		} 
		 
		
		
		f.createNewFile();
		f.setWritable(true);
		System.out.println("f exists" + f.exists() + f.setWritable(true));
		writer = new PrintWriter(f);		
		if(writer==null) System.out.println("Writer is null");
		writer.println("Vernier Format 2");
		writer.println("Untiled.clmb 5/5/2019 9:37:43 .");
		writer.println("Data Set");
		writer.println("Time	Input	Output	Error");
		writer.println("x	y	z	w\n");
	
	}
	
	public void writeNewData(double seconds, double input, double output, double error) {
		writer.println("" + seconds + "\t" + input + "\t" + output + "\t" + error);
	}

	public void flushData(){
		writer.flush();
	}
	

}
