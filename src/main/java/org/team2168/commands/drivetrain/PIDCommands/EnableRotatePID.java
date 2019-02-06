
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author shriji
 */
public class EnableRotatePID extends Command {

    public EnableRotatePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    
	protected void initialize() {
		
		Robot.drivetrain.rotateController.Enable();
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		
    	
    }

    //delete me
    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    }
}
