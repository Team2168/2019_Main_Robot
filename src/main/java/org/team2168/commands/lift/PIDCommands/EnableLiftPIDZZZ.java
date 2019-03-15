
package org.team2168.commands.lift.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author Vittorio
 */
public class EnableLiftPIDZZZ extends Command {

	private double setPoint;
	private double maxSpeed;
	private double minSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends.
	private boolean absolute = true;
	
    public EnableLiftPIDZZZ() {
        // Use requires() here to declare subsystem dependencies
    	requires(Lift.getInstance());
    	this.setPoint = Robot.lift.liftPOTController.getSetPoint();
    	this.maxSpeed = 0.5;
		this.minSpeed = 0;
		this.absolute = true;
    }

    public EnableLiftPIDZZZ(double setPoint){
 	   this();
 	   this.setPoint = setPoint;
    }

    public EnableLiftPIDZZZ(double setPoint, double maxSpeed){
  	   this(setPoint);
  	   this.maxSpeed = maxSpeed;
     }
    
    public EnableLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed){
   	   this(setPoint, maxSpeed);
   	   this.minSpeed = minSpeed;
      }    

    public EnableLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error) {
    	this(setPoint, maxSpeed, minSpeed);
    	this.error = error;
    	this.absolute = false;
    }
    
    public EnableLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute) {
    	this(setPoint, maxSpeed, minSpeed, error);
    	this.absolute = absolute;
    }
    // Called just before this Command runs the first time
    
	protected void initialize() {
		double sp = 0;
		if (!absolute)
			sp = this.setPoint + Robot.lift.getPotPos();
		else
			sp = this.setPoint;
		Robot.lift.liftPOTController.reset();

//		Robot.turret.rotateTurretPOTController.setpGain(RobotMap.ROTATE_POSITION_P);
//		Robot.turret.rotateTurretPOTController.setiGain(RobotMap.ROTATE_POSITION_I);
//		Robot.turret.rotateTurretPOTController.setdGain(RobotMap.ROTATE_POSITION_D);
		Robot.lift.liftPOTController.setSetPoint(sp);
		Robot.lift.liftPOTController.setMaxPosOutput(maxSpeed);
		Robot.lift.liftPOTController.setMaxNegOutput(-maxSpeed);
		Robot.lift.liftPOTController.setMinPosOutput(minSpeed);
		Robot.lift.liftPOTController.setMinNegOutput(-minSpeed);
		Robot.lift.liftPOTController.setAcceptErrorDiff(error);
		//Robot.drivetrain.gyroSPI.reset();
		Robot.lift.liftPOTController.Enable();
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		
		Robot.lift.driveAllMotors(Robot.lift.liftPOTController.getControlOutput());
	
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.lift.liftPOTController.Pause();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}