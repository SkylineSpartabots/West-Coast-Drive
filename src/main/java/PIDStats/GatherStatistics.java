package PIDStats;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.Date;

public class GatherStatistics {
	private PrintWriter writer;
	private Date time;
	private int count = 0;
	private boolean bigMode;
	public enum PID_TYPE {TURN, LEFT_ENCODER, RIGHT_ENCODER, ENCODER}
	public DecimalFormat decimalFormat;
	public GatherStatistics(PID_TYPE pType, boolean bigMode) throws IOException {
		decimalFormat = new DecimalFormat("#.000");

		this.bigMode = bigMode;
		if (bigMode) {
			count++;
			time = new Date();
			File f;
			if(pType == PID_TYPE.TURN)	{
				f = new File("/home/lvuser/deploy/TurnPIDStats/" + time.toString() +".txt");
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
			writer.println("Time	Input	Error	Integral	Derivative	RawOutput	Output");
			writer.println("t	in	e	i	d	r	o");
			writer.println();
			writer.println();

		} else {
		count++;
		time = new Date();
		File f;
		if(pType == PID_TYPE.TURN)	{
			f = new File("/home/lvuser/deploy/TurnPIDStats/" + time.toString() +".txt");
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
		writer.println("x	y	z	w");
		writer.println();
		}
	
	}
	
	public void writeNewData(double seconds, double input, double output, double error) {
		writer.println("" + seconds + "\t" + input + "\t" + output + "\t" + error);
	}

	public void writeLotsOfData(double seconds, double input, double error, double integral, double derivative, double rawOutput, double output) {
		writer.println("" + decimalFormat.format(seconds) + "\t" + decimalFormat.format(input) + "\t"  + decimalFormat.format(error) + "\t"  + decimalFormat.format(integral) + "\t"  + decimalFormat.format(derivative) + "\t"  + decimalFormat.format(rawOutput) + "\t"  + decimalFormat.format(output));
	}

	public void flushData(){
		writer.flush();
	}
	

}
