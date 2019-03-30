
package org.team2168.commands.hatchProbePivot.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;
import org.team2168.subsystems.HatchProbePivot;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 *
 * @author Vittorio
 */
public class DriveHatchProbePivotPathPIDZZZ extends Command {

	
	private double[] pos;
	private double[] vel;
	private double[] accel;
    
	private double setPoint;
	OneDimensionalMotionProfiling motion;
	int counter;
	double ff_term = 0.15;

	private double maxSpeed;
	private double minSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends. 
	
	public final double MAX_VEL = 90;
	public final double MAX_ACCEL = 93;
	public final double MAX_JERK = 1000;
	
    public DriveHatchProbePivotPathPIDZZZ() {
        // Use requires() here to declare subsystem dependencies
		requires(HatchProbePivot.getInstance());
		this.setPoint = Robot.hatchProbePivot.hatchProbePivotController.getSetPoint();
    	this.maxSpeed = 1;
		this.minSpeed = 0;
		
		//SmartDashboard.putNumber("FF_term_HP", ff_term);
    }

    public DriveHatchProbePivotPathPIDZZZ(double setPoint){
 	   	this();
		this.setPoint = setPoint; 
    }
    
	protected void initialize() 
	{
		motion = new OneDimensionalMotionProfiling(Robot.hatchProbePivot.getPotPos(),setPoint,this.MAX_VEL,this.MAX_ACCEL,this.MAX_JERK);

		this.pos = motion.pos;
		this.vel = motion.vel;

		
		Robot.hatchProbePivot.hatchProbePivotController.reset();
		
		Robot.hatchProbePivot.hatchProbePivotController.setpGain(RobotMap.HP_PIVOT_P);
		Robot.hatchProbePivot.hatchProbePivotController.setiGain(RobotMap.HP_PIVOT_I);
		Robot.hatchProbePivot.hatchProbePivotController.setdGain(RobotMap.HP_PIVOT_D);
		Robot.hatchProbePivot.hatchProbePivotController.setSetPoint(this.pos);
		Robot.hatchProbePivot.hatchProbePivotController.setMaxPosOutput(maxSpeed);
		Robot.hatchProbePivot.hatchProbePivotController.setMaxNegOutput(-maxSpeed);
		Robot.hatchProbePivot.hatchProbePivotController.setMinPosOutput(minSpeed);
		Robot.hatchProbePivot.hatchProbePivotController.setMinNegOutput(-minSpeed);
		Robot.hatchProbePivot.hatchProbePivotController.setAcceptErrorDiff(error);
		
		Robot.hatchProbePivot.hatchProbePivotController.Enable();

		counter = 0;

		System.out.print("Lenght: "+ pos.length);
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		//ff_term = SmartDashboard.getNumber("FF_term_HP", 0);
		
		if (counter < pos.length)
		{
		  double pidSpeed = Robot.hatchProbePivot.hatchProbePivotController.getControlOutput();
		  double ff_Speed = (ff_term  * vel[counter]) / (Robot.pdp.getBatteryVoltage());
		  Robot.hatchProbePivot.drivePlungerArmPivotMotor(ff_Speed+pidSpeed);
		  //System.out.println(ff_Speed+pidSpeed);
		}
		else
		  Robot.hatchProbePivot.drivePlungerArmPivotMotor(0.0);
		  counter++;
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
    	return (counter >= pos.length) || (Robot.hatchProbePivot.getPotPos() < pos[pos.length-1]+1 && Robot.hatchProbePivot.getPotPos() > pos[pos.length-1]) ;
		//return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.hatchProbePivot.hatchProbePivotController.Pause();
		Robot.hatchProbePivot.drivePlungerArmPivotMotor(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}