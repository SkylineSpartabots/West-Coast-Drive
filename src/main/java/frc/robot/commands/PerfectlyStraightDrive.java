package frc.robot.commands;

import frc.robot.Robot;
import util.PIDSource;
import util.SimplePID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.Map;

import com.sun.javadoc.RootDoc;

public class PerfectlyStraightDrive extends Command {

  

  private final static double COUNTS_PER_REV = 1000;
  private final static double WHEEL_DIAMETER = 4;
  private final static double DRIVE_WHEEL_REDUCTION = 1;
  private final static double COUNTS_PER_INCH = (COUNTS_PER_REV/DRIVE_WHEEL_REDUCTION)/(WHEEL_DIAMETER*Math.PI);

  private double distanceInches;
  private boolean direction;

  private double leftPower, rightPower;

  private double rightTarget, leftTarget;

	private double angle;
	private boolean isFinished = false;
	private double error;
	
	private double output = 0;
	PIDSource NavxSource;
	SimplePID turnPID;

  public double kP = 0.025;
	public double kI = 0.00001;
	public double kD = 0.00135;



  public PerfectlyStraightDrive(double distanceInches, double leftPower, double rightPower) throws IOException {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distanceInches = distanceInches;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    requires(Robot.drive);
    angle = Robot.angle;
    //angle = 180;
    
		NavxSource = new PIDSource(){
			@Override
			public double getInput() {
				return Robot.navx.getAngle();
			}
		};

    SmartDashboard.putNumber("kP, Perfect Drive", kP);
    //perfectlyStraightOuput();
		//SmartDashboard.
    
   // kP = Shuffleboard.getTab("PID").add("PValue", 0.02).withWidget(BuiltInWidgets.kTextView).getEntry().getValue().getDouble();

    
		turnPID = new SimplePID(NavxSource, this.angle, kP, kI, kD);
		turnPID.setOutputLimits(-0.5, 0.5);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //rightTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    leftTarget = COUNTS_PER_INCH * this.distanceInches + Robot.drive.leftEncoder.getRaw();
    Robot.drive.tankDrive(0, 0);
    this.direction = distanceInches > 0;
    turnPID.resetPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //perfectlyStraightOuput();
    output = turnPID.compute();
    Robot.drive.tankDrive(leftPower - output, rightPower + output);
    //System.out.println(Robot.drive.m_midLeft.get());
    //System.out.println(Robot.drive.leftEncoder.getRaw());
    
    Robot.drive.sensorOutput();
    System.out.println(output);
    /*if (this.angle <= Robot.navx.getAngle()) {
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
  public void perfectlyStraightOuput() {
    //SmartDashboard.putNumber("kP, Perfect Drive", kP);

    kP = SmartDashboard.getNumber("kP, Perfect Drive", 0.02);
  } 
}