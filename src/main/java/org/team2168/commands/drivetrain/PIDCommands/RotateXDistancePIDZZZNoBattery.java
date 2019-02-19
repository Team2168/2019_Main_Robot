
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author Vittorio
 */
public class RotateXDistancePIDZZZNoBattery extends Command {

	private double setPoint;
	private double maxSpeed;
	private double minSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends.
	private boolean absolute = false;
	
    public RotateXDistancePIDZZZNoBattery() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drivetrain);
    	this.setPoint = Robot.drivetrain.rotateController.getSetPoint();
    	this.maxSpeed = 1;
    	this.minSpeed = 0;
    }

    public RotateXDistancePIDZZZNoBattery(double setPoint){
 	   this();
 	   this.setPoint = setPoint;
    }

    public RotateXDistancePIDZZZNoBattery(double setPoint, double maxSpeed){
  	   this(setPoint);
  	   this.maxSpeed = maxSpeed;
     }
    
    public RotateXDistancePIDZZZNoBattery(double setPoint, double maxSpeed, double minSpeed){
   	   this(setPoint, maxSpeed);
   	   this.minSpeed = minSpeed;
      }    

    public RotateXDistancePIDZZZNoBattery(double setPoint, double maxSpeed, double minSpeed, double error) {
    	this(setPoint, maxSpeed, minSpeed);
    	this.error = error;
    	this.absolute = false;
    }
    
    public RotateXDistancePIDZZZNoBattery(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute) {
    	this(setPoint, maxSpeed, minSpeed, error);
    	this.absolute = absolute;
    }
    // Called just before this Command runs the first time
    
	protected void initialize() {
		double sp = 0;
		if (!absolute)
			sp = this.setPoint + Robot.drivetrain.getHeading();
		else
			sp = this.setPoint;
		Robot.drivetrain.rotateController.reset();

//		Robot.drivetrain.rotateController.setpGain(RobotMap.ROTATE_POSITION_P);
//		Robot.drivetrain.rotateController.setiGain(RobotMap.ROTATE_POSITION_I);
//		Robot.drivetrain.rotateController.setdGain(RobotMap.ROTATE_POSITION_D);
		Robot.drivetrain.rotateController.setSetPoint(sp);
		Robot.drivetrain.rotateController.setMaxPosOutput(maxSpeed);
		Robot.drivetrain.rotateController.setMaxNegOutput(-maxSpeed);
		Robot.drivetrain.rotateController.setMinPosOutput(minSpeed);
		Robot.drivetrain.rotateController.setMinNegOutput(-minSpeed);
		Robot.drivetrain.rotateController.setAcceptErrorDiff(error);
		//Robot.drivetrain.gyroSPI.reset();
		Robot.drivetrain.rotateController.Enable();
		
		System.out.println(sp);
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {

		//Set Min as minimum voltage to drive mechanims based on emperical measurements. Should account for battery dipping. Calculate Motor controller %
		//Robot.drivetrain.rotateController.setMinPosOutput(RobotMap.DRIVE_TRAIN_MIN_ROT_CLOCKWISE_VOLTAGE/Robot.pdp.getBatteryVoltage());
		//Robot.drivetrain.rotateController.setMinNegOutput(-RobotMap.DRIVE_TRAIN_MIN_ROT_COUNTCLOCKWISE_VOLTAGE/Robot.pdp.getBatteryVoltage());
		
		Robot.drivetrain.tankDrive(Robot.drivetrain.rotateController.getControlOutput(),-Robot.drivetrain.rotateController.getControlOutput());
    
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
		//TODO Should the command be stopped????????!?!?!?!?!? after PID is tuned
    	return Robot.drivetrain.rotateController.isFinished();
		//return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.drivetrain.rotateController.Pause();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}