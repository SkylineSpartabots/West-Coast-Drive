package frc.robot.commands;

import frc.robot.Robot;
import util.PIDSource;
import util.SimplePID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.Map;

import com.sun.javadoc.RootDoc;


public class TurnPIDEncoderDrive extends Command {

  

  private final static double COUNTS_PER_REV = 1000;
  private final static double WHEEL_DIAMETER = 4;
  private final static double DRIVE_WHEEL_REDUCTION = 1;
  private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER*Math.PI);

  private double distanceInches;
  private boolean direction;

  private double leftPower, rightPower;

  private double rightTarget, leftTarget;

	private double angle=90;
	private boolean isFinished = false;
	private double error;
  private Timer timer;
	private double output = 0;
	PIDSource NavxSource;
  SimplePID turnPID;
  

	double kP = 0.005;
	double kI = 0.0001; //0.00001
	double kD = 0; //0.00135  might wanna put

  public TurnPIDEncoderDrive(double distanceInches, double leftPower, double rightPower) throws IOException {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    requires(Robot.drive);
    timer = new Timer();
    //angle = Robot.angle;
   // angle = 180;
    SmartDashboard.putNumber("target angle", angle);
    SmartDashboard.putNumber("navx start point", Robot.navx.getAngle());
		NavxSource = new PIDSource(){
			@Override
			public double getInput() {
				return Robot.navx.getAngle();
			}
		};

		//SmartDashboard.
    
   // kP = Shuffleboard.getTab("PID").add("PValue", 0.02).withWidget(BuiltInWidgets.kTextView).getEntry().getValue().getDouble();

    
		turnPID = new SimplePID(NavxSource, this.angle, kP, kI, kD, timer, true);
		turnPID.setOutputLimits(-0.2, 0.2);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.reset();
    timer.start();
    //rightTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    leftTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    Robot.drive.tankDrive(0, 0);
    this.direction = distanceInches > 0;
    turnPID.resetPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    output = turnPID.compute();
    error = turnPID.getError();

   
    Robot.drive.tankDrive(leftPower + output, rightPower - output);
    //System.out.println(Robot.drive.m_midLeft.get());
    //System.out.println(Robot.drive.leftEncoder.getRaw());
    
    

    Robot.drive.sensorOutput();
    //System.out.println(output);
    SmartDashboard.putNumber("output for pid", output);
  /*  if (this.angle <= Robot.navx.getAngle()) {
      System.out.println("HEY IM AT THE ANGLE U WANT ME TO BE AT ####################################################################################################");
    }*/

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(direction){
      if(leftTarget <= Robot.drive.leftEncoder.getRaw()){
        turnPID.resetPID();
        return true;
      } else{
        return false;
      }
        
    } else{
      if(leftTarget >= Robot.drive.leftEncoder.getRaw()){
        turnPID.resetPID();
        return true;
      } else{
        return false;
      }
    }
    // || rightTarget >= Robot.drive.rightEncoder.getRaw();
      
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.tankDrive(0, 0);
    timer.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
}