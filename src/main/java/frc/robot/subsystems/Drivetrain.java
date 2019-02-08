/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoyStick;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  private DifferentialDrive m_drive;

  public DriveTrain() {
    Victor rightFrontMotor = new Victor(RobotMap.RIGHT_FRONT_DRIVE_MOTOR);
    Victor leftFrontMotor = new Victor(RobotMap.LEFT_FRONT_DRIVE_MOTOR);
    Victor rightBackMotor = new Victor(RobotMap.RIGHT_BACK_DRIVE_MOTOR);
    Victor leftBackMotor = new Victor(RobotMap.LEFT_BACK_DRIVE_MOTOR);

    SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);
    SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
    leftSide.setInverted(true);

    m_drive = new DifferentialDrive(leftSide, rightSide);
  }

  public void arcadeDrive(double speed, double rotate) {
    m_drive.arcadeDrive(speed, rotate);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithJoyStick(1, 4));
  }
}
