
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;



/**
 *
 * @author shriji
 */
public class DrivePIDPause extends Command {

    public DrivePIDPause() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    
	protected void initialize() {
		Robot.drivetrain.rightSpeedController.Pause();
		Robot.drivetrain.leftSpeedController.Pause();
		Robot.drivetrain.rotateController.Pause();
		Robot.drivetrain.driveTrainPosController.Pause();
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
    	
    }

    //delete me
    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
        return (Robot.drivetrain.rightSpeedController.isEnabled() == false) 
        		&& (Robot.drivetrain.driveTrainPosController.isEnabled() == false)
        		&& (Robot.drivetrain.leftSpeedController.isEnabled() == false)
        		&& (Robot.drivetrain.rotateController.isEnabled() == false);
    }

    // Called once after isFinished returns true
    
	protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    }
}
