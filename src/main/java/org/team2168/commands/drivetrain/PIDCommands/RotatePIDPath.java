
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RotatePIDPath extends Command {
	
	private double[] setPointLeft;
    private double[] setPointRight;
    private double[] setPointHeading;
    
    OneDimensionalMotionProfiling motion;
	
    int counter;
    double ff_term = 1.11;
    double oldClock;
    double angle;
    double lastRotateOutput;
    boolean direction = false;
    int directionValue = 1;
    private boolean headingByArray = false;
    
    public RotatePIDPath(double distance )
    {
    	this(distance,false);
    }
    
    public RotatePIDPath(double distance, boolean reverseDirection )
    {
    	requires(Robot.drivetrain);
    	motion = new OneDimensionalMotionProfiling(distance,12.0,8.0,30.0);
    	
  	   this.setPointLeft =  motion.getVelArray();
  	   this.setPointRight = motion.getVelArray();
  	   this.setPointHeading = motion.getPosArray();
  	   this.direction = reverseDirection;
  	   this.headingByArray = true;
    }
   
    

    // Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.leftSpeedController.reset();
		Robot.drivetrain.leftSpeedController.Enable();
		Robot.drivetrain.leftSpeedController.setSetPoint(setPointLeft);
		
		Robot.drivetrain.rightSpeedController.reset();
		Robot.drivetrain.rightSpeedController.Enable();
		Robot.drivetrain.rightSpeedController.setSetPoint(setPointRight);
		if(this.headingByArray)
			Robot.drivetrain.rotateDriveStraightController.setSetPoint(setPointHeading);
    
		Robot.drivetrain.rotateDriveStraightController.reset();
		counter = 0;
		oldClock = Timer.getFPGATimestamp();
		
		
		Robot.drivetrain.resetPosition();

		//reset controller
		Robot.drivetrain.imu.reset();
		Robot.drivetrain.driveTrainPosController.reset();
		Robot.drivetrain.rotateDriveStraightController.reset();

		angle = Robot.drivetrain.getHeading();
		this.lastRotateOutput = 0;
		
		Robot.drivetrain.rotateDriveStraightController.Enable();
		
		//if true we want to reverse else we want to go forward
		if (direction)
			directionValue = -1;
		else
			directionValue = 1;
    }

    // Called repeatedly when this Command is scheduled to run
    
	protected void execute() 
	{
		//Robot.drivetrain.tankDrive(Robot.drivetrain.leftSpeedController.getControlOutput(),
    	//Robot.drivetrain.rightSpeedController.getControlOutput());
        
		double currTime = Timer.getFPGATimestamp(); 
		SmartDashboard.putNumber("Command Execution Time", (currTime - oldClock));
		oldClock = currTime;
		
		//ff_term = SmartDashboard.getNumber("FF_term", 0);
		
		lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
		double headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput()) ;
		
		if(counter<setPointLeft.length)
		{
			double speedLeft = (ff_term*directionValue*setPointLeft[counter])/(Robot.pdp.getBatteryVoltage());
			double speedRight = (ff_term*directionValue*setPointRight[counter])/(Robot.pdp.getBatteryVoltage());
			if (Math.abs(speedLeft)<0.12 && counter!=0)
				speedLeft = directionValue*0.12;
			
			if (Math.abs(speedRight)<0.12 && counter!=0)
				speedRight = directionValue*0.12;
			
			Robot.drivetrain.tankDrive(speedLeft+headingCorrection,speedRight-headingCorrection);
			counter++;
			
			SmartDashboard.putNumber("DriveArrayLeftSpeed", speedLeft);
			SmartDashboard.putNumber("DriveArrayRightSpeed", speedRight);
			SmartDashboard.putNumber("DriveArrayHeading", Robot.drivetrain.rotateDriveStraightController.getSetPoint());
		}

    }

    // Make this return true when this Command no longer needs to run execute()
    
    
	protected boolean isFinished() {
        return (!Robot.drivetrain.leftSpeedController.isSetPointByArray() &&  Robot.drivetrain.leftSpeedController.isFinished()) && (!Robot.drivetrain.rightSpeedController.isSetPointByArray() &&  Robot.drivetrain.rightSpeedController.isFinished());
    }

    // Called once after isFinished returns true
    
	protected void end() {
		Robot.drivetrain.leftSpeedController.Pause();
		Robot.drivetrain.rightSpeedController.Pause();
		Robot.drivetrain.rotateDriveStraightController.Pause();
		Robot.drivetrain.tankDrive(0, 0);
		
    }

    //delete me
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}
