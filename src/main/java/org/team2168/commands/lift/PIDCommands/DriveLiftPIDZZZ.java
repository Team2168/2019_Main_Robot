
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
	

	private double maxPosSpeed;
	private double maxNegSpeed;
	private double minPosSpeed;
	private double minNegSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends. 
	private boolean absolute = true;
	
    public DriveLiftPIDZZZ() {
        
    	requires(Lift.getInstance());
    	this.setPoint = Robot.lift.liftPOTController.getSetPoint();
    	this.maxPosSpeed = 1;
		this.minPosSpeed = 0;
		this.maxNegSpeed = -1;
    	this.minNegSpeed = 0;
    }

    public DriveLiftPIDZZZ(double setPoint){
 	   this();
 	  this.setPoint = setPoint;
    }

    public DriveLiftPIDZZZ(double setPoint, double maxPosSpeed){
  	   this(setPoint);
  	   this.maxPosSpeed = maxPosSpeed;
     }
    
    public DriveLiftPIDZZZ(double setPoint, double maxPosSpeed, double minPosSpeed){
   	   this(setPoint, maxPosSpeed);
   	   this.minPosSpeed = minPosSpeed;
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
	
	public DriveLiftPIDZZZ(double setPoint, double maxPosSpeed, double maxNegSpeed, double minPosSpeed, double minNegSpeed, double error, boolean absolute) {
    	this(setPoint);
		this.absolute = absolute;
		this.error = error;
		this.maxNegSpeed = maxNegSpeed;
		this.maxPosSpeed = maxPosSpeed;
		this.minNegSpeed = minNegSpeed;
		this.minPosSpeed = minPosSpeed;
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
		Robot.lift.liftPOTController.setMaxPosOutput(maxPosSpeed);
		Robot.lift.liftPOTController.setMaxNegOutput(maxNegSpeed);
		Robot.lift.liftPOTController.setMinPosOutput(minPosSpeed);
		Robot.lift.liftPOTController.setMinNegOutput(minNegSpeed);
		Robot.lift.liftPOTController.setAcceptErrorDiff(error);

		Robot.lift.liftPOTController.enableHoldingVoltage(RobotMap.LIFT_HOLDING_VOLTAGE);

		
		Robot.lift.liftPOTController.Enable();
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		
		Robot.lift.driveAllMotors(Robot.lift.liftPOTController.getControlOutput());
	
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
		//TODO Should the command be stopped????????!?!?!?!?!? after PID is tuned
    	return Robot.lift.liftPOTController.isFinished() || Math.abs(Robot.oi.getLiftJoystickValue()) > 0.05;
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