package org.team2168.commands.pneumatics;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Starts the compressor
 * 
 * @author Ben Waid
 */
public class StartCompressor extends Command {

	public StartCompressor() {
		requires(Robot.pneumatics);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.pneumatics.startCompressor();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
