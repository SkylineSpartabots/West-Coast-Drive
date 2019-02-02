/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.NavX;
import util.PIDSource;
import util.SimplePID;

import java.io.FileNotFoundException;
import java.io.IOException;

import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class TurnDegrees extends Command {
	private int clockCounter=0;
	private double angle;
	private final double CLOCK_MAX = 5;
	private boolean isFinished = false;
	private double error;
	//private GatherStatistics stats;
	private Timer timer;
	private double output = 0;
	PIDSource NavxSource;
	SimplePID turnPID;

	double kP = 0.055;
	double kI = 0.00001;
	double kD = 0.00135;
	
	

	/*
		double kP = 0.229;
	double kI = 0.308;
	double kD = 0.205;
	0.00135
	*/
	
	
	public TurnDegrees(double angle) throws IOException {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drive);
		timer = new Timer();
		this.angle = angle + Robot.navx.getAngle();
	/*	try {
			stats = new GatherStatistics(PID_TYPE.TURN);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/

		NavxSource = new PIDSource(){
		
			@Override
			public double getInput() {
				return Robot.navx.getAngle();
			}
		};

		//SmartDashboard.
		
		turnPID = new SimplePID(NavxSource, this.angle, kP, kI, kD, timer, true);
		turnPID.setOutputLimits(-0.6, 0.6);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		timer.reset();
		timer.start();
		turnPID.resetPID();
		clockCounter = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		error = turnPID.getError();
		output = turnPID.compute();

	
		if (Math.abs(error) < Robot.drive.threshold) {
			clockCounter++;
			if (clockCounter >= CLOCK_MAX) {
				isFinished = true;
			}
		} else {
			clockCounter = 0;
		}
		
		Robot.drive.tankDrive(-output, output);
		Robot.drive.sensorOutput();
		//stats.writeNewData(timer.get(), Robot.navx.getAngle(), output, turnPID.getError());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		turnPID.resetPID();
		Robot.drive.tankDrive(0, 0);
		timer.stop();
	//	stats.flushData();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
