
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

	
    private double[] setPointLift;
    
    OneDimensionalMotionProfiling motion;
	int counter;
    double ff_term = 1.11;
	private double maxSpeed;
	private double minSpeed;
	private double error = 0.5;  // Rotational degree error, default 0 never ends. 
	private boolean absolute = false;
	private boolean direction = false; //false is forward
	int directionValue = 1;
	
    public DriveLiftPathPIDZZZ() {
        // Use requires() here to declare subsystem dependencies
    	requires(Lift.getInstance());
    	this.maxSpeed = 1;
    	this.minSpeed = 0;
    }

    public DriveLiftPathPIDZZZ(double setPoint){
 	   this();
 	  motion = new OneDimensionalMotionProfiling(setPoint);
 	 this.setPointLift =  motion.getVelArray();
 	SmartDashboard.putNumber("FF_term_Lift", 1.11);
	   
    }
    public DriveLiftPathPIDZZZ(double setPoint, double v_max, double a_max, double j_max){
  	   this();
  	  motion = new OneDimensionalMotionProfiling(setPoint, v_max, a_max, j_max);
  	 this.setPointLift =  motion.getVelArray();
  	SmartDashboard.putNumber("FF_term_Lift", 1.11);
 	   
     }

    public DriveLiftPathPIDZZZ(double setPoint, double v_max, double a_max, double j_max, boolean direction){
   	   this();
   	  motion = new OneDimensionalMotionProfiling(setPoint, v_max, a_max, j_max);
   	 this.setPointLift =  motion.getVelArray();
   	SmartDashboard.putNumber("FF_term_Lift", 1.11);
   	this.direction = direction;
  	   
      }
    public DriveLiftPathPIDZZZ(double setPoint, double maxSpeed){
  	   this(setPoint);
  	   this.maxSpeed = maxSpeed;
     }
    
    public DriveLiftPathPIDZZZ(double setPoint, double maxSpeed, double minSpeed){
   	   this(setPoint, maxSpeed);
   	   this.minSpeed = minSpeed;
      }    
    public DriveLiftPathPIDZZZ(double setPoint, double maxSpeed, double minSpeed, boolean direction ){
    	   this(setPoint, maxSpeed);
    	   this.minSpeed = minSpeed;
    	   this.direction = direction;
       }
    
//    public DriveLiftPathPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error) {
//    	this(setPoint, maxSpeed, minSpeed);
//    	this.error = error;
//    	this.absolute = false;
//    	this.direction = true;
//    }
//    
//    public DriveLiftPathPIDZZZ(double setPoint, double maxSpeed, double minSpeed, double error, boolean absolute) {
//    	this(setPoint, maxSpeed, minSpeed, error);
//    	this.absolute = absolute;
//    	this.direction = true;
//    }
    // Called just before this Command runs the first time
    
	protected void initialize() {
		double sp = 0;
		Robot.lift.liftPOTController.reset();
		counter = 0;
		Robot.lift.liftPOTController.setpGain(RobotMap.LIFT_P);
		Robot.lift.liftPOTController.setiGain(RobotMap.LIFT_I);
		Robot.lift.liftPOTController.setdGain(RobotMap.LIFT_D);
		Robot.lift.liftPOTController.setSetPoint(sp);
		Robot.lift.liftPOTController.setMaxPosOutput(maxSpeed);
		Robot.lift.liftPOTController.setMaxNegOutput(-maxSpeed);
		Robot.lift.liftPOTController.setMinPosOutput(minSpeed);
		Robot.lift.liftPOTController.setMinNegOutput(-minSpeed);
		Robot.lift.liftPOTController.setAcceptErrorDiff(error);
		
		Robot.lift.liftPOTController.Enable();
		ff_term = SmartDashboard.getNumber("FF_term_Lift", 1.11);
		//if true we want to reverse else we want to go forward
				if (direction)
					directionValue = -1;
				else
					directionValue = 1;
		
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() {
		
		if(counter<setPointLift.length) 
		{
			double liftSpeed = (ff_term*directionValue*setPointLift[counter])/(Robot.pdp.getBatteryVoltage());
			Robot.lift.driveAllMotors(liftSpeed); 
			counter++; 
		}
		
    }

    // Make this return true when this Command no longer needs to run execute()
    
	protected boolean isFinished() {
		//TODO Should the command be stopped????????!?!?!?!?!? after PID is tuned
    	return (Robot.lift.liftPOTController.isFinished() && Robot.lift.liftPOTController.isSetPointByArray());
		//return false;
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.lift.liftPOTController.Pause();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}