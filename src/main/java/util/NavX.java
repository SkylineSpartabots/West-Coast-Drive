package util;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import frc.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import java.util.Timer;
import java.util.TimerTask;


public class NavX {
	public AHRS ahrs;

	
	public NavX() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}

	public void reset() {
	 ahrs.reset();
	}

	// Will return degrees
	public double getAngle() {
		return ahrs.getAngle();
	}

}