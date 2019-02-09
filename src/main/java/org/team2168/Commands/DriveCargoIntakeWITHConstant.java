package org.team2168.Commands;

import edu.wpi.first.wpilibj.command.Command;
import org.team2168.robot.Robot;

/**
 * The purpose of this class is to control the CargoIntake with code instead of joysticks/controller.
 */
public class DriveCargoIntakeWITHConstant extends Command {

	int counter;
	double speed;

	public DriveCargoIntakeWITHConstant(double speed) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.cargointake);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		counter = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.cargointake.drive(speed);
		counter ++;
		//adds 1 counter then, goes to boolean, and since its not greater than 200 its gonna return false and come back here and add 1 more each time, until its greaqter than 200 than its gonna return true
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.cargointake.drive(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
