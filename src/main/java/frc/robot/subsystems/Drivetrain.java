/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoyStick;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands. 

  WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(RobotMap.leftFrontMotor);
  WPI_TalonSRX m_midLeft = new WPI_TalonSRX(RobotMap.leftMidMotor);
  WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(RobotMap.leftBackMotor);
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_midLeft, m_rearLeft);

  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(RobotMap.rightFrontMotor);
  WPI_TalonSRX m_midRight = new WPI_TalonSRX(RobotMap.rightMidMotor);
  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(RobotMap.rightBackMotor);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_midRight, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoyStick());
  }
  public void arcadeDrive(double xSpeed, double zRotation){
    m_drive.arcadeDrive(xSpeed, zRotation);
  }
}