/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.Map;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import util.*;
import frc.robot.commands.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot  {
  //public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static Drivetrain drive;
  public static Object DriveTrain;
  public static NavX navx;
  public static double angle;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private static final ShuffleboardTab TAB = Shuffleboard.getTab("WestCoast");
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    drive = new Drivetrain();
    navx = new NavX();
    m_chooser.setDefaultOption("Default Auto", new DriveWithJoyStick());
    // chooser.addOption("My Auto", new MyAutoCommand());
    try {
      m_chooser.addOption("Test Turning", new TestAuto());
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    m_chooser.addOption("Left Encoder Only", new LeftEncoderAuto());
    SmartDashboard.putData("Auto mode", m_chooser);
    //LiveWindow.add(drive);
    navx.reset();
    angle = Robot.navx.getAngle();

    TAB.add("Left Encoder", drive.leftEncoder.getRaw());
    TAB.add("Magnetic Limit Switch", drive.limitSwitch.get());
    TAB.add("Navx", navx.getAngle());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Shuffleboard.update();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_chooser.getSelected();
    navx.reset();
    drive.leftEncoder.reset();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
   // m_autonomousCommand = new LeftEncoderAuto();
    try {
      m_autonomousCommand = new TestAuto();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
	}
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    drive.sensorOutput();
    //drive.neoOuput();
    if(!Robot.drive.limitSwitch.get()){
      Robot.drive.leftEncoder.reset();
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
