package util;


import java.util.TimerTask;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
/**
 * PID Controller Object
 * <p>
 * Starts a runnable class PIDCompute that executes the PID algorithm
 * <p>
 * Method Signature: public PIDMain(Object pidsource,int setpoint, int
 * sampleTime, double kp, double ki, double kd)
 * 
 * @author NeilHazra
 */

// The heart of PID
public class SimplePID {
	private boolean enabled = true; // allows the pid algorithm to stop
									// computing
	private double outputMax = 1;
	private double outputMin = -1;
	private PIDSource pidsource;
	private double output; // value to send to the motor
	/** The process error */
	private double error;
	private double rawOutput;
	/** The input of the PIDController */
	private double input; // what the value actually is
	private double setpoint; // desired target value
	private double prevInput;
	private double prevError;
	private double integral; // I term
	private double derivative;// D term
	public double kp, ki, kd; // tuning parameters, the hardest part of PID
	private double prevTime = 0;
	/**
	 * @param pidsource
	 *            Object implementing PIDSource, contains method returning input
	 * @param setpoint
	 *            target value for PID controller
	 * @param sampleTime
	 *            time between successive calculations
	 * @param kp
	 *            proportional gain
	 * @param ki
	 *            integral gain
	 * @param kd
	 *            derivative gain
	 */
	private PrintWriter writer;
	private Date time;
	public enum PID_TYPE {TURN, LEFT_ENCODER, RIGHT_ENCODER, ENCODER}
	public DecimalFormat decimalFormat;
	private boolean doStats;
	private File f;
	private Timer timer;

	
	
	/**
	 * @return Use for stats-tracking PID instances, be sure to make timer before you construct this class. Por favor.
	 * @param pidsource
	 * @param setpoint
	 * @param kp
	 * @param ki
	 * @param kd
	 * @param timer
	 * @param doStats
	 * @throws IOException
	 */
	public SimplePID(Object pidsource, double setpoint, double kp, double ki, double kd, Timer timer, boolean doStats) throws IOException{
		this.pidsource = (PIDSource) pidsource;
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.doStats = doStats;
		this.timer = timer;

		if (this.doStats) {
			time = new Date();
			f = new File("/home/lvuser/deploy/PIDStats/" + this.getClass().getName() + " " + time.toString() +".txt");
			
			this.getClass().getName();
			
			
			f.createNewFile();
			f.setWritable(true);
			System.out.println("f exists" + f.exists() + f.setWritable(true));
			writer = new PrintWriter(f);		
			if(writer==null) System.out.println("Writer is null");
			writer.println("Vernier Format 2");
			writer.println("Untiled.clmb 5/5/2019 9:37:43 .");
			writer.println("Data Set");
			writer.println("Time	Input	Output	Error");
			writer.println("s	i	o	e");
			writer.println();
		}

	}

	/**
	 * @return Use for NON-stats tracking PID instances
	 * @param pidsource
	 * @param setpoint
	 * @param kp
	 * @param ki
	 * @param kd
	 * @throws IOException
	 */

	public SimplePID(Object pidsource, double setpoint, double kp, double ki, double kd) throws IOException{
		this.pidsource = (PIDSource) pidsource;
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
	}

	
	
	public double getOutput() {
		return output;
	}

	public static double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax) {
		return (newMax - newMin) * (oldValue - oldMin) / (oldMax - oldMin) + newMin;
	}

	// For debug and tuning
	public double getInput() {
		return pidsource.getInput();
	}

	public double getError() {
		return error;
	}

	/**
	 * @return the desired target value
	 */
	public double getSetpoint() {
		return setpoint;
	}

	/**
	 * Sets the desired target value
	 * 
	 * @param setpoint
	 *            desired target value
	 */
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		resetPID();
	}

	public void setOutputLimits(double min, double max) {
		outputMax = max;
		outputMin = min;
	}

	/**
	 * PID Algorithm calculates in this TimerTask created by PIDMain
	 * 
	 * @author NeilHazra
	 *
	 */
	public void resetPID() {
		integral = 0;
		derivative = 0;
		writer.flush();
	}

	public double compute()	{
		long currentTime = System.currentTimeMillis();
		double dt = (currentTime - prevTime)/1000.0;
		input = pidsource.getInput();
		error = setpoint - input;
		if (error < 1 && error > -1) {
			integral +=  error;
		} else {
			integral = 0;
		}
		
		// constrains integral in between outputMin and outputMax
		/*if (integral > outputMax) {
			integral = outputMax;
		}
		if (integral < outputMin) {
			integral = outputMin;
		}
		*/
		
		//derivative = kd * (input - prevInput) / dt;
		derivative = (error - prevError) / dt;

		output = kp * error + ki*integral + kd*derivative;

		rawOutput = output;
		// constrains output in between outputMin and outputMax
		if (output > outputMax) {
			output = outputMax;
		}
		if (output < outputMin) {
			output = outputMin;
		}
		prevInput = input;
		prevError = error;
		prevTime = currentTime;

		if (this.doStats) {
			
			writer.println("" + this.timer.get() + "\t" + input + "\t" + output + "\t" + error);

		}

		return output;
	}	

	public double getDerivative() {
		return derivative;
	}
	public double getIntegral() {
		return integral;
	}
	public double getRawOutput() {
		return rawOutput;
	}
}