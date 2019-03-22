package org.team2168.commands.cargoIntake;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeUntilCargo extends Command {

    public IntakeUntilCargo() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cargoIntakeWheels);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.cubeIntakeGripper.extendIntake(); 
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(!Robot.cargoIntakeWheels.isCargoPresent())
    		Robot.cargoIntakeWheels.drive(-RobotMap.CARGO_INTAKE_MAX_SPEED);
    	
    	//Robot.i2c.write(8, 5);
    		 
    }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.cargoIntakeWheels.isCargoPresent();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.cargoIntakeWheels.drive(0.0);
    	//Robot.cubeIntakeGripper.retractIntake();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.cargoIntakeWheels.drive(0.0);
    }
}
