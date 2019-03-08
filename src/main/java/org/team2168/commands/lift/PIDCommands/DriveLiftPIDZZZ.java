
package org.team2168.commands.lift.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author Vittorio
 */
public class DriveLiftPIDZZZ extends Command {

	
	private double setPoint;
	

	private double maxSpeed;
	private double minSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends. 
	private boolean absolute = true;
	
    public DriveLiftPIDZZZ() {
        
    	requires(Lift.getInstance());
    	this.setPoint = Robot.lift.liftPOTController.getSetPoint();
    	this.maxSpeed = 1;
    	this.minSpeed = 0;
    }

    public DriveLiftPIDZZZ(double setPoint){
 	   this();
 	  this.setPoint = setPoint;
    }

    public DriveLiftPIDZZZ(double setPoint, double maxSpeed){
  	   this(setPoint);
  	   this.maxSpeed = maxSpeed;
     }
    
    public DriveLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed){
   	   this(setPoint, maxSpeed);
   	   this.minSpeed = minSpeed;
      }    

    public DriveLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error) {
    	this(setPoint, maxSpeed, minSpeed);
    	this.error = error;
    	this.absolute = false;
    }
    
    public DriveLiftPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute) {
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

		Robot.lift.liftPOTController.setpGain(RobotMap.LIFT_P);
		Robot.lift.liftPOTController.setiGain(RobotMap.LIFT_I);
		Robot.lift.liftPOTController.setdGain(RobotMap.LIFT_D);
		Robot.lift.liftPOTController.setSetPoint(sp);
		Robot.lift.liftPOTController.setMaxPosOutput(maxSpeed);
		Robot.lift.liftPOTController.setMaxNegOutput(-maxSpeed);
		Robot.lift.liftPOTController.setMinPosOutput(minSpeed);
		Robot.lift.liftPOTController.setMinNegOutput(-minSpeed);
		Robot.lift.liftPOTController.setAcceptErrorDiff(error);
		
		Robot.lift.liftPOTController.Enable();
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		
		Robot.lift.driveAllMotors(Robot.lift.liftPOTController.getControlOutput());
	
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
		//TODO Should the command be stopped????????!?!?!?!?!? after PID is tuned
    	return Robot.lift.liftPOTController.isFinished();
		//return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.lift.liftPOTController.Pause();
		Robot.lift.driveAllMotors(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}