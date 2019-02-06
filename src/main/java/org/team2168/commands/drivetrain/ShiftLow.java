package org.team2168.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;

import org.team2168.robot.Robot;

/**
 * Shifts the drivetrain into low gear.
 */
public class ShiftLow extends Command {

    public ShiftLow() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrainShifter);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrainShifter.shiftToLow();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.drivetrainShifter.inLowGear();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
