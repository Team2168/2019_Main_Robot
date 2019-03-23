
package org.team2168.commands.lift.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;
import org.team2168.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 *
 * @author Vittorio
 */
public class DriveLiftPathPIDZZZ extends Command {

	
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
	
    public DriveLiftPathPIDZZZ() {
        // Use requires() here to declare subsystem dependencies
		requires(Lift.getInstance());
		this.setPoint = Robot.lift.liftPOTController.getSetPoint();
    	this.maxSpeed = 1;
		this.minSpeed = 0;
		
		//SmartDashboard.putNumber("FF_term_Lift", ff_term);
    }

    public DriveLiftPathPIDZZZ(double setPoint){
 	   	this();
		this.setPoint = setPoint; 
    }
    
	protected void initialize() 
	{
		motion = new OneDimensionalMotionProfiling(Robot.lift.getPotPos(),setPoint,this.MAX_VEL,this.MAX_ACCEL,this.MAX_JERK);

		this.pos = motion.pos;
		this.vel = motion.vel;

		
		Robot.lift.liftPOTController.reset();
		
		Robot.lift.liftPOTController.setpGain(RobotMap.LIFT_P);
		Robot.lift.liftPOTController.setiGain(RobotMap.LIFT_I);
		Robot.lift.liftPOTController.setdGain(RobotMap.LIFT_D);
		Robot.lift.liftPOTController.setSetPoint(this.pos);
		Robot.lift.liftPOTController.setMaxPosOutput(maxSpeed);
		Robot.lift.liftPOTController.setMaxNegOutput(-maxSpeed);
		Robot.lift.liftPOTController.setMinPosOutput(minSpeed);
		Robot.lift.liftPOTController.setMinNegOutput(-minSpeed);
		Robot.lift.liftPOTController.setAcceptErrorDiff(error);
		
		Robot.lift.liftPOTController.Enable();

		counter = 0;

		System.out.print("Lenght: "+ pos.length);
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		//ff_term = SmartDashboard.getNumber("FF_term_Lift", 0);
		
		if (counter < pos.length)
		{
		  double pidSpeed = Robot.lift.liftPOTController.getControlOutput();
		  double ff_Speed = (ff_term  * vel[counter]) / (Robot.pdp.getBatteryVoltage());
		  Robot.lift.driveAllMotors(ff_Speed+pidSpeed);
		  //System.out.println(ff_Speed+pidSpeed);
		}
		else
		  Robot.lift.driveAllMotors(0.0);
		  counter++;
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
    	return (counter >= pos.length) || (Robot.lift.getPotPos() < pos[pos.length-1]+1 && Robot.lift.getPotPos() > pos[pos.length-1]) ;
		//return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.lift.liftPOTController.Pause();
		Robot.lift.driveAllMotors(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}