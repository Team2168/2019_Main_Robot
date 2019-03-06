
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author shriji
 */
public class DriveRightPIDSpeed extends Command {
	
	private double setPoint;

    public DriveRightPIDSpeed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	this.setPoint = 0;
    }
    
   public DriveRightPIDSpeed(double setPoint){
	   this();
	   this.setPoint = setPoint;
	   
   }

    // Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.rightSpeedController.reset();
		Robot.drivetrain.rightSpeedController.Enable();
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
    	if (setPoint != 0)
    		Robot.drivetrain.rightSpeedController.setSetPoint(setPoint);
    	Robot.drivetrain.driveRight(Robot.drivetrain.rightSpeedController.getControlOutput());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
        return Robot.drivetrain.rightSpeedController.isEnabled() == false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.drivetrain.rightSpeedController.Pause();
    }

    //delete me
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}
