package org.team2168.commands.cargoIntake;

import edu.wpi.first.wpilibj.command.Command;

import org.team2168.Robot;

/**
 * The purpose of this class is to drive the CargoIntake with Joysticks
 */
public class DriveCargoIntakeWithJoystick extends Command
{

	public DriveCargoIntakeWithJoystick()
	{
		// Use requires() here to declare subsystem dependencies
		requires(Robot.cargoIntakeWheels);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize()
	{

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute()
	{
		Robot.cargoIntakeWheels.drive(Robot.oi.getDriveCargoIntakeJoystickValue());

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished()
	{
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end()
	{
		Robot.cargoIntakeWheels.drive(0.0);// 0.0 because its a double
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted()
	{
		end();
	}
}
