
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author shriji
 */
public class DriveLeftPIDSpeed extends Command {
	
	private double setPoint;

    public DriveLeftPIDSpeed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	this.setPoint = 0;
    }
    
   public DriveLeftPIDSpeed(double setPoint){
	   this();
	   this.setPoint = setPoint;
	   
   }

    // Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.leftSpeedController.reset();
		Robot.drivetrain.leftSpeedController.Enable();
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
    	if (setPoint != 0)
    		Robot.drivetrain.leftSpeedController.setSetPoint(setPoint);
    	Robot.drivetrain.driveLeft(Robot.drivetrain.leftSpeedController.getControlOutput());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
        return Robot.drivetrain.leftSpeedController.isEnabled() == false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.drivetrain.leftSpeedController.Pause();
    }

    //delete me
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}
