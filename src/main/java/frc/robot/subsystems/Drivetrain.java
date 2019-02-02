/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoyStick;
import frc.robot.Robot;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private WPI_TalonSRX m_frontLeft;
  public WPI_TalonSRX m_midLeft;
  private WPI_TalonSRX m_rearLeft;
  private SpeedControllerGroup m_left;

  private WPI_TalonSRX m_frontRight;
  private WPI_TalonSRX m_midRight;
  private WPI_TalonSRX m_rearRight;
  private SpeedControllerGroup m_right;

  private DifferentialDrive m_drive;
  public Encoder leftEncoder;
  public Encoder rightEncoder;
  
  double kP = 0.002;
	double kD = 0.0;
  double kI = 0.0;
  
  private static final int leftEncoderSourceA = 0;
  private static final int leftEncoderSourceB = 1;
  private static final int rightEncoderSourceA = 4;
  private static final int rightEncoderSourceB = 5;

  public DigitalInput limitSwitch = null;
  
  public double threshold = 2;

  private static final ShuffleboardTab TAB = Shuffleboard.getTab("WestCoast");

  //public CANSparkMax neoMotor;
  //public CANEncoder neoEncoder; //42 counts per rev
  public static final double countsPerRevHallEncoder = 42;
  //https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
//link for revrobotics libraries
  public Drivetrain() {
    leftEncoder = new Encoder(leftEncoderSourceA, leftEncoderSourceB);
    rightEncoder = new Encoder(rightEncoderSourceA, rightEncoderSourceB);

    WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(RobotMap.leftFrontMotor);
    WPI_TalonSRX m_midLeft = new WPI_TalonSRX(RobotMap.leftMidMotor);
    WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(RobotMap.leftBackMotor);
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_midLeft.setNeutralMode(NeutralMode.Brake);
    m_rearLeft.setNeutralMode(NeutralMode.Brake);

    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_midLeft, m_rearLeft);

    WPI_TalonSRX m_frontRight = new WPI_TalonSRX(RobotMap.rightFrontMotor);
    WPI_TalonSRX m_midRight = new WPI_TalonSRX(RobotMap.rightMidMotor);
    WPI_TalonSRX m_rearRight = new WPI_TalonSRX(RobotMap.rightBackMotor);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_midRight.setNeutralMode(NeutralMode.Brake);
    m_rearRight.setNeutralMode(NeutralMode.Brake);

    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_midRight, m_rearRight);

    m_drive = new DifferentialDrive(m_left, m_right);
    limitSwitch = new DigitalInput(7);


    /*TAB.add("Left Encoder", leftEncoder.getRaw());
    TAB.add("Magnetic Limit Switch", limitSwitch.get());
    TAB.add("Navx", Robot.navx.getAngle());

    Shuffleboard.update();
    */
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoyStick());
  }
  public void arcadeDrive(double xSpeed, double zRotation){
    m_drive.arcadeDrive(xSpeed, zRotation);
    
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /*public void neoMotorSetPower(double power){
    neoMotor.set(power);
  }*/

  public void neoOuput() {
    //SmartDashboard.putNumber("Neo Motor Power", neoMotor.get());
    //SmartDashboard.putNumber("Neo Motor Encoder", neoEncoder.getPosition());
  }

  public void sensorOutput() {
    String s;
    s = "left encoder: " + leftEncoder.getRaw() + " |||| right encoder: " + rightEncoder.getRaw(); 

    SmartDashboard.putNumber("Left Encoder Raw", leftEncoder.getRaw());
    SmartDashboard.putNumber("Left Encoder Distance", rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder Get", rightEncoder.get());

    SmartDashboard.putBoolean("Magnetic Limit Switch", limitSwitch.get());

    SmartDashboard.putNumber("NavX", Robot.navx.getAngle());
    SmartDashboard.putNumber("Left Encoder Source A", leftEncoderSourceA);
    SmartDashboard.putNumber("Left Encoder Source B", leftEncoderSourceB);
    SmartDashboard.putNumber("Right Encoder Source A", rightEncoderSourceA);
    SmartDashboard.putNumber("Right Encoder Source B", rightEncoderSourceB);
    SmartDashboard.updateValues();
    Shuffleboard.update();
  //  return s;
  }
}