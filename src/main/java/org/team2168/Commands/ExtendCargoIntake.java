package org.team2168.Commands;

import edu.wpi.first.wpilibj.command.Command;
import org.team2168.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ExtendCargoIntake extends Command {
	public ExtendCargoIntake() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.cargointake);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

        Robot.cargointake.intakeOpen(); //shouldnt have double because i dont think you woul dhave double for a piston that extends etc
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.cargointake.cargoRetracted()//we want it to be retracted instead of extended because it should retract once its already extended, i think
;	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
