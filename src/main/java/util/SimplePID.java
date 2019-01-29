package util;

import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	/** The input of the PIDController */
	private double input; // what the value actually is
	private double setpoint; // desired target value
	private double prevInput;
	private double proportional; // P term
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
	public SimplePID(Object pidsource, double setpoint, double kp, double ki, double kd) {
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
		proportional = 0;
		integral = 0;
		derivative = 0;
	}

	public double compute()	{
		long currentTime = System.currentTimeMillis();
		double dt = (currentTime - prevTime)/1000.0;
		input = pidsource.getInput();
		error = input - setpoint;
		proportional = kp * error;
		integral += ki * error;
		// constrains integral in between outputMin and outputMax
		if (integral > outputMax) {
			integral = outputMax;
		}
		if (integral < outputMin) {
			integral = outputMin;
		}
		
		
		derivative = kd * (input - prevInput) / dt;

		output = proportional + integral + derivative;
		// constrains output in between outputMin and outputMax
		if (output > outputMax) {
			output = outputMax;
		}
		if (output < outputMin) {
			output = outputMin;
		}
		prevInput = input;
		prevTime = currentTime;
		return output;
	}	
}