package org.team2168.Commands;

import edu.wpi.first.wpilibj.command.Command;

import org.team2168.robot.OI;
import org.team2168.robot.Robot;

/**
 * The purpose of this class is to drive the CargoIntake with Joysticks
 */
public class DriveCargoIntakeJoystick extends Command {


    
	public DriveCargoIntakeJoystick() {
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
		Robot.cargointake.drive(OI.getDriveCargoIntakeJoystickValue());

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.cargointake.drive(0.0);//0.0 because its a double
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
