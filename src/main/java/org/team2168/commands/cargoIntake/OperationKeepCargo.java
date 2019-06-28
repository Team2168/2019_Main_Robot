package org.team2168.commands.cargoIntake;

import org.team2168.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OperationKeepCargo extends Command {

    public OperationKeepCargo() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cargoIntakeWheels);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.cubeIntakeGripper.extendIntake(); 
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.cargoIntakeWheels.getRawIRVoltage()<3.1 && Robot.cargoIntakeWheels.getRawIRVoltage()>2.5)
        {
            Robot.cargoIntakeWheels.drive(-0.25);
        }
        else if(Robot.cargoIntakeWheels.getRawIRVoltage()<2.5)
        {
            Robot.cargoIntakeWheels.drive(-1.0);
        }
        else
        {
            Robot.cargoIntakeWheels.drive(-0.075);
        }
    	
    	//Robot.i2c.write(8, 5);
    		 
    }
    

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.operatorJoystick.getLeftTriggerAxisRaw() > 0.1;
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
