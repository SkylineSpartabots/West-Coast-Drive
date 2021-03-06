/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import util.SimplePID;

import java.io.IOException;

import frc.robot.Robot;


/**
 * An example command.  You can replace me with your own command.
 */
public class LeftEncoderDrive extends Command {

	private double threshold = 50;
	private double clockCounter;
	private static double CLOCK_MAX = 5;
	private boolean isFinished;

	private double distanceInches;
	//private util.PIDSource rightSource;
	private util.PIDSource leftSource;
	
	
	//private SimplePID rightPID;
	//private double rightError;
	//private double rightOutput;

	private SimplePID leftPID;
	private double leftError;
	private double leftOutput;

	private double rightTarget, leftTarget;

	///private GatherStatistics stats;
	private Timer timer;

    private final static double COUNTS_PER_REV = 1000;
    private final static double WHEEL_DIAMETER = 4;//idk but looks four
    private final static double DRIVE_WHEEL_REDUCTION = 1;
    private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER/Math.PI);

	private double kP = 0.008;
	private double kI = 0;
	private double kD = 0;

	public LeftEncoderDrive(double distanceInches) throws IOException {
		// Use requires() here to declare subsystem dependencies
        requires(Robot.drive);
		this.distanceInches = distanceInches;


	//	try {
		//	stats = new GatherStatistics(PID_TYPE.ENCODER);
	//	} catch (IOException e) {
	//		e.printStackTrace();
	//	}

	/*	rightSource = new util.PIDSource(){
			@Override
			public double getInput() {
				return Robot.drive.rightEncoder.getRaw();
			}
		};*/
		
		leftSource = new util.PIDSource(){
			@Override
			public double getInput() {
				return Robot.drive.leftEncoder.getRaw();
			}
		};


		//rightTarget = Robot.drive.rightEncoder.getRaw() + (distanceInches * COUNTS_PER_INCH);
		leftTarget = Robot.drive.leftEncoder.getRaw() + (distanceInches * COUNTS_PER_INCH);

		//rightPID = new SimplePID(rightSource, rightTarget, kP, kI, kD);
		//rightPID.setOutputLimits(-1, 1);

		leftPID = new SimplePID(leftSource, leftTarget, kP, kI, kD, timer, true);
		leftPID.setOutputLimits(-0.5, 0.5);

		timer = new Timer();
		
	}

	// Called just before this Command rs the first time
	@Override
	protected void initialize() {
		timer.reset();
		timer.start();
		//rightPID.resetPID();
		leftPID.resetPID();
		clockCounter = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//rightError = rightPID.getError();
		leftError = leftPID.getError();
		
		//rightOutput = rightPID.compute();
		leftOutput = leftPID.compute();

		if (Math.abs(leftError) < threshold){
			clockCounter++;
			if(clockCounter >= CLOCK_MAX){
				isFinished = true;
			}
		} else{
			clockCounter = 0;
		}

		Robot.drive.tankDrive(-leftOutput, -leftOutput);
	//	stats.writeNewData(timer.get(), Robot.drive.leftEncoder.getRaw(), leftOutput, leftError);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		//rightPID.resetPID();
		leftPID.resetPID();
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
