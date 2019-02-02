/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveWithJoyStick extends Command {
  private double turn;
  private double forward;
  private double leftTriggerPower;
  private double rightTriggerPower;
public DriveWithJoyStick() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.arcadeDrive(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    turn = Robot.m_oi.stick.getRawAxis(OI.RX);
    forward = Robot.m_oi.stick.getRawAxis(OI.LY)*0.75;
    Robot.drive.arcadeDrive(-forward, turn);

    leftTriggerPower = Robot.m_oi.stick.getRawAxis(OI.LTrigger);
    rightTriggerPower= Robot.m_oi.stick.getRawAxis(OI.RTrigger);
    //Robot.drive.neoMotorSetPower(leftTriggerPower - rightTriggerPower); //sets neo motor to the total of the left and right trigger, the right trigger is negative and left is positive
    //System.out.println(Robot.drive.sensorOutput());
    Robot.drive.sensorOutput();
    Robot.drive.neoOuput();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.arcadeDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
