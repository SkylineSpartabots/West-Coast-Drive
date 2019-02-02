/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import util.SimplePID;

import java.io.IOException;

import frc.robot.Robot;


public class EllipticalDriveToPoint extends Command {
	private double x;
	private double y;

	private double leftc;
	private double lefta;
	private double rightc;
	private double righta;
	private double leftb;
	private double rightb;
	
	private double halfRobot = 12.25;

  	private double threshold = 50;
	private double clockCounter;
	private static double CLOCK_MAX = 5;
	private boolean isFinished;

	private util.PIDSource rightSource;
	private util.PIDSource leftSource;
	
	
	private SimplePID rightPID;
	private double rightError;
	private double rightOutput;

	private SimplePID leftPID;
	private double leftError;
	private double leftOutput;

	private double rightTarget, leftTarget;

  	private Timer timer;
  

  	private final static double COUNTS_PER_REV = 1000;
  	private final static double WHEEL_DIAMETER = 8;
  	private final static double DRIVE_WHEEL_REDUCTION = 1;
  	private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER/Math.PI);

	private double kP = 0.008;
	private double kI = 0;
	private double kD = 0;
  public EllipticalDriveToPoint(double x, double y) throws IOException { // just made x and y make it so that you can
																			// find the eliptical path for left and
																			// right, use the witdth of the robot and
																			// extend the radii of the ellipse
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    	requires(Robot.drive);

		this.x = x;
		this.y = y;

		
		rightSource = new util.PIDSource(){
			@Override
			public double getInput() {
				return Robot.drive.rightEncoder.getRaw();
			}
		};
		
		leftSource = new util.PIDSource(){
			@Override
			public double getInput() {
				return Robot.drive.leftEncoder.getRaw();
			}
		};
		leftb = this.y - halfRobot;
		lefta = this.x - halfRobot; 
		leftc = (Math.PI*(lefta+leftb))*((3*Math.pow(2, lefta-leftb))/(Math.pow(2, lefta+leftb)*(Math.pow(0.5, (-3*(Math.pow(2, lefta-leftb)/Math.pow(2, lefta+leftb))+4))+10) + 1));

		rightb = this.y + halfRobot;
		rightb = this.x + halfRobot; 
		rightc = (Math.PI*(righta+rightb))*((3*Math.pow(2, righta-rightb))/(Math.pow(2, righta+rightb)*(Math.pow(0.5, (-3*(Math.pow(2, righta-rightb)/Math.pow(2, righta+rightb))+4))+10) + 1));

		
		rightTarget = Robot.drive.rightEncoder.getRaw() + (rightc * COUNTS_PER_INCH);
		leftTarget = Robot.drive.leftEncoder.getRaw() + (leftc * COUNTS_PER_INCH);

		rightPID = new SimplePID(rightSource, rightTarget, kP, kI, kD, timer, true);
		rightPID.setOutputLimits(-1, 1);

		leftPID = new SimplePID(leftSource, leftTarget, kP, kI, kD, timer, true);
		leftPID.setOutputLimits(-1, 1);

		timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
	Robot.drive.tankDrive(0, 0);
	timer.reset();
	timer.start();
	rightPID.resetPID();
	leftPID.resetPID();
	clockCounter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
	rightError = rightPID.getError();
	leftError = leftPID.getError();
	
	rightOutput = rightPID.compute();
	leftOutput = leftPID.compute();
	  
	if(Math.abs(rightError) < threshold || Math.abs(leftError) < threshold){
		clockCounter++;
		if(clockCounter >= CLOCK_MAX){
			isFinished = true;
		}
	} else{
		clockCounter = 0;
	}

	Robot.drive.tankDrive((leftc/rightc)*leftOutput, (rightc/leftc)*rightOutput);

  }//width of bot, 24.5 in
  //half 12.25

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
	  Robot.drive.tankDrive(0, 0);
	  leftPID.resetPID();
	  rightPID.resetPID();
	  timer.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
	  end();
  }
}
