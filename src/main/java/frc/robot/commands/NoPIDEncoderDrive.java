/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class NoPIDEncoderDrive extends Command {

  

  private final static double COUNTS_PER_REV = 1000;
  private final static double WHEEL_DIAMETER = 4;
  private final static double DRIVE_WHEEL_REDUCTION = 1;
  private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER*Math.PI);

  private double distanceInches;
  private boolean direction;

  private double leftPower, rightPower;

  private double rightTarget, leftTarget;

  public NoPIDEncoderDrive(double distanceInches, double leftPower, double rightPower) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //rightTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    leftTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    Robot.drive.tankDrive(0, 0);
    this.direction = distanceInches > 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.tankDrive(leftPower, rightPower);
    System.out.println(Robot.drive.m_midLeft.get());
    System.out.println(Robot.drive.leftEncoder.getRaw());

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(direction){
        return leftTarget <= Robot.drive.leftEncoder.getRaw();
    } else{
        return leftTarget >= Robot.drive.leftEncoder.getRaw();
    }
    // || rightTarget >= Robot.drive.rightEncoder.getRaw();
      
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.tankDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
}
