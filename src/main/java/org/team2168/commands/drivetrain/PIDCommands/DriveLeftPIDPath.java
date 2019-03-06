
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;


public class DriveLeftPIDPath extends Command {
	
	private double[][] setPoint;

   public DriveLeftPIDPath(double[][] setPoint){
	   requires(Robot.drivetrain);
	   this.setPoint = setPoint;
	   
   }

    // Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.leftSpeedController.reset();
		Robot.drivetrain.leftSpeedController.Enable();
		Robot.drivetrain.leftSpeedController.setSetPoint(setPoint);
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
    	Robot.drivetrain.driveLeft(Robot.drivetrain.leftSpeedController.getControlOutput());
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
        return !Robot.drivetrain.leftSpeedController.isSetPointByArray() &&  Robot.drivetrain.leftSpeedController.isFinished();
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
