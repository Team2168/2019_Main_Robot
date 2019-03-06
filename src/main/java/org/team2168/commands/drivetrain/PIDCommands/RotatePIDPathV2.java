
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RotatePIDPathV2 extends Command {
	
	private double[] setPointLeft;
    private double[] setPointRight;
    private double[] setPointHeadingOriginal;
    private double[] setPointHeadingDriven;
    
    private double[] setAccelPointLeft;
    private double[] setAccelPointRight;
    
    OneDimensionalMotionProfiling motion;
    
    private double finalPosition;
	
    int counter;
    double ff_term_rotate = 0.2;
    double ff_a_term_rotate = 0.2;
    double oldClock;
    double angle;
    double lastRotateOutput;
    boolean direction = false;
    int directionValue = 1;
    
    private boolean headingByArray = false;
    
    public RotatePIDPathV2(double distance )
    {
    	this(distance,false);
    }
    
    public RotatePIDPathV2(double distance, boolean reverseDirection )
    {
    	requires(Robot.drivetrain);
    	motion = new OneDimensionalMotionProfiling(distance, 600.0,2400.0,30000.0);
  	   this.setPointLeft =  motion.getVelArray();
  	   this.setPointRight = motion.getVelArray();
  	   this.direction = reverseDirection;
  	   this.finalPosition = distance;
  	    this.setAccelPointLeft = motion.getAccelArray();
  	    this.setAccelPointRight = motion.getAccelArray();
  	   
  	   this.setPointHeadingOriginal = motion.getPosArray();
  	   
  	   //this.setPointHeadingDriven = new double[setPointHeadingOriginal.length];
  	   
	  
	   this.headingByArray= true;
	   
	   SmartDashboard.putNumber("FF_term_rotate", 0);
	   SmartDashboard.putNumber("FF_A_term_rotate", 0);

	   
	   System.out.println("SetPointLength: " + setPointLeft.length);
    }
   
   
    
   
    // Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.leftSpeedController.reset();
		Robot.drivetrain.leftSpeedController.Enable();
		Robot.drivetrain.leftSpeedController.setSetPoint(setPointLeft);
		
		Robot.drivetrain.rightSpeedController.reset();
		Robot.drivetrain.rightSpeedController.Enable();
		Robot.drivetrain.rightSpeedController.setSetPoint(setPointRight);
		
		//reset controller
		Robot.drivetrain.imu.reset();
		Robot.drivetrain.driveTrainPosController.reset();
		Robot.drivetrain.rotateController.reset();
    
		angle = Robot.drivetrain.getHeading();
		System.out.println("Drivetrain Angle: " + angle);
		motion = new OneDimensionalMotionProfiling(angle,this.finalPosition, 420.0,190.0,300.0);
		
	  	   this.setPointLeft =  motion.getVelArray();
	  	   this.setPointRight = motion.getVelArray();

	  	    this.setAccelPointLeft = motion.getAccelArray();
	  	    this.setAccelPointRight = motion.getAccelArray();
	  	   
	  	   this.setPointHeadingOriginal = motion.getPosArray();
		
	  	   this.setPointHeadingDriven = new double[this.setPointHeadingOriginal.length];
	  	   
	  	 System.out.println("Drivetrain length: " + setPointHeadingDriven.length);
		for(int i=0; i<setPointHeadingOriginal.length; i++) {
			setPointHeadingDriven[i] = setPointHeadingOriginal[i];
			System.out.println("Heading: " + setPointHeadingDriven[i]);
			
		}
				
		
		
		Robot.drivetrain.rotateController.reset();
		if(this.headingByArray)
			Robot.drivetrain.rotateController.setSetPoint(setPointHeadingDriven);
		
		Robot.drivetrain.rotateController.Enable();
		
		
		counter = 0;
		oldClock = Timer.getFPGATimestamp();
		
		
		Robot.drivetrain.resetPosition();



		
		this.lastRotateOutput = 0;
		
		Robot.drivetrain.rotateController.Enable();
		
		
		
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
		
		ff_term_rotate = SmartDashboard.getNumber("FF_term_rotate", 0);
		ff_a_term_rotate = SmartDashboard.getNumber("FF_A_term_rotate", 0);
		
		lastRotateOutput = Robot.drivetrain.rotateController.getControlOutput();
		double headingCorrection = (Robot.drivetrain.rotateController.getControlOutput()) ;
		
		if(counter<setPointLeft.length)
		{
			double speedLeft = ((ff_term_rotate*directionValue*setPointLeft[counter])/Robot.pdp.getBatteryVoltage());
			double speedRight = ((ff_term_rotate*-directionValue*setPointRight[counter])/Robot.pdp.getBatteryVoltage());
			
		
			
			Robot.drivetrain.tankDrive(speedLeft+headingCorrection,speedRight-headingCorrection);
			counter++;
			
			SmartDashboard.putNumber("DriveArrayLeftSpeed", speedLeft);
			SmartDashboard.putNumber("DriveArrayRightSpeed", speedRight);
			SmartDashboard.putNumber("DriveArrayHeading", Robot.drivetrain.rotateController.getSetPoint());
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
		Robot.drivetrain.rotateController.Pause();
		Robot.drivetrain.tankDrive(0, 0);
		
    }

    //delete me
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
	protected void interrupted() {
    	end();
    }
}
