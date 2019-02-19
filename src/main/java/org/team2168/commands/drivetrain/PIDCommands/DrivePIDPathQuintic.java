
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivePIDPathQuintic extends Command {
	
	private double[] setPointLeft;
    private double[] setPointRight;
    private double[] setPointHeading;
    
    OneDimensionalMotionProfiling motion;
	OneDimensionalMotionProfiling wheels;
	OneDimensionalMotionProfiling angleRot;
	
//	double vMax = 2500.0;
//	double aMax = 3000.0;
//	double jMax =30000.0;
	
	double vMax = 300.0;
	double aMax = 2000.0;
	double jMax =15000.0;
	
    int counter;
    double ff_term = 1.15;
    double oldClock;
    double angle;
    double lastRotateOutput;
    boolean direction = false;
    int directionValue = 1;
    
    private boolean headingByArray = false;
    private boolean rotateInPlace = false;
    double finalRotDistance;
    
    public DrivePIDPathQuintic(double distance )
    {
    	this(distance,false);
    }
    
    public DrivePIDPathQuintic(double distance, boolean reverseDirection )
    {
    	requires(Robot.drivetrain);
    	motion = new OneDimensionalMotionProfiling(distance);
  	   this.setPointLeft =  motion.getVelArray();
  	   this.setPointRight = motion.getVelArray();
  	   this.direction = reverseDirection;
    }
   
    public DrivePIDPathQuintic(double[] setPointLeft, double[] setPointRight){
 	   requires(Robot.drivetrain);
 	 
 	   this.setPointLeft =setPointLeft;
 	   this.setPointRight = setPointRight;
 	   
 	   direction = false;
 	   
 	   System.out.println("SetPointLength: " + setPointLeft.length);
    } 
    
    public DrivePIDPathQuintic(double[] setPointLeft, double[] setPointRight,  double[] setPointHeading){
    	   requires(Robot.drivetrain);
    	 
    	   this.setPointLeft =setPointLeft;
    	   this.setPointRight = setPointRight;
    	   this.setPointHeading = setPointHeading;
    	   
    	   direction = false;
    	   this.headingByArray= true;

    	   
    	   System.out.println("SetPointLength: " + setPointLeft.length);
       } 
    
    
    public DrivePIDPathQuintic(double[] setPointLeft, double[] setPointRight,  double[] setPointHeading, boolean direction){
    	   requires(Robot.drivetrain);
    	 
    	   this.direction = direction;
    	   
    	   this.setPointLeft =setPointLeft;
    	   this.setPointRight = setPointRight;
    	   this.setPointHeading = setPointHeading;
    	   

    	   if(!direction)
    	   {
    		   this.setPointLeft = setPointLeft;
     	   this.setPointRight = setPointRight;
     	   this.setPointHeading = setPointHeading;
    		   
    	   }
    	   //we want to drive the path backwards
    	   // swap the left and right wheels, and negate the velocitys, also correct
    	   //heading to be 180 from current position
    	   else
    	   {
    		   this.setPointLeft = setPointRight;
    	   	   this.setPointRight = setPointLeft;
    	   	   this.setPointHeading = setPointHeading;
    	   	   
    	   	   for (int i=0; i<this.setPointHeading.length; i++)
    	   	   {
    	   		   this.setPointHeading[i] = 180+this.setPointHeading[i];
    	   	   }
    	   }
    	   
    	   this.headingByArray= true;

    	   
    	   System.out.println("SetPointLength: " + setPointLeft.length);
       }
    
    
    
    
    public DrivePIDPathQuintic(double start, double distance, double v_max, double accel_max, double j_max)
	{
    	requires(Robot.drivetrain);
    	
    	this.finalRotDistance = distance;
    	rotateInPlace = true;
		double circumference;
		if(distance>start) //rotate clockwise
		{
			circumference = (Math.PI*3*(distance-start)/180.0)/12.0;
			angleRot = new OneDimensionalMotionProfiling( start,  distance,  v_max,  accel_max,  j_max);
		}
		else
		{
			circumference = (Math.PI*3*(start-distance)/180.0)/12.0;
			angleRot = new OneDimensionalMotionProfiling( distance,  start,  v_max,  accel_max,  j_max);
			this.direction = true;
		}

		
		angleRot.S_curves();
		System.out.println(circumference);
		wheels = new OneDimensionalMotionProfiling(circumference, 8.0, 8.0, 100.0);
		wheels.S_curves();
		
		
		
		this.setPointLeft = new double[wheels.getVelArray().length];
		this.setPointRight = new double[wheels.getVelArray().length];
		
		for (int i = 0; i<setPointLeft.length; i++)
		{
			this.setPointLeft[i] = wheels.getVelArray()[i];
			this.setPointRight[i] = -wheels.getVelArray()[i];
		}
		
		
		int counterAngle = angleRot.time.length-1;
		int counterWheels = wheels.time.length-1;
		
		this.setPointHeading = new double[wheels.getTimeArray().length];
		
		//if angle array larger
		if(counterAngle >= counterWheels)
		{
			
			
			for(int i=setPointHeading.length-1; i>=0; i--)
			{
				setPointHeading[i] = angleRot.pos[counterAngle];
				counterAngle--;
			}
		}	
		else //wheels is greater than angle
		{
			for(int i=counterAngle; i>=0; i-- )
			{
				setPointHeading[counterWheels] = angleRot.pos[i];
				counterWheels--;
			}
			
			for(int i=counterWheels; i>=0; i--)
				setPointHeading[i]=angleRot.pos[0];;
			
			System.out.println("that");
		}
 	   
		   this.headingByArray= true;
	 	   this.rotateInPlace = true;
	 	   
 	  if(!direction)
	   {
		   this.setPointLeft = setPointLeft;
		   this.setPointRight = setPointRight;
		   this.setPointHeading = setPointHeading;
		   
	   }
	   //we want to drive the path backwards
	   // swap the left and right wheels, and negate the velocitys, also correct
	   //heading to be 180 from current position
	   else //invert heading
	   {
		   this.setPointLeft = setPointRight;
	   	   this.setPointRight = setPointLeft;
	   	   this.setPointHeading = setPointHeading;
	   	   
	   	   double[] temp = new double[this.setPointHeading.length];
	   	   int counter = this.setPointHeading.length-1;
	   	   for (int i=0; i<temp.length; i++)
	   	   {
	   		   temp[i] = this.setPointHeading[counter];
	   		   counter--;
	   	   }
	   	   
	   	   this.setPointHeading = temp;
	   }
 	   
 	  this.ff_term = 1.1;
 	   System.out.println("SetPointLength: " + setPointLeft.length);
	
	}
    
    public DrivePIDPathQuintic(double[] setPointLeft, double[] setPointRight, double ff_gain){
  	   requires(Robot.drivetrain);
  	   this.setPointLeft = setPointLeft;
  	   this.setPointRight = setPointRight;
  	   ff_term = ff_gain;
  	   
  	   direction = false;
  	   
  	   
     } 
    
   public DrivePIDPathQuintic(double[] setPointLeft, double[] setPointRight, boolean reverseDirection){
	   requires(Robot.drivetrain);
	   this.setPointLeft = setPointLeft;
	   this.setPointRight = setPointRight;
	   SmartDashboard.putNumber("FF_term", 0);
	   ff_term = SmartDashboard.getNumber("FF_term", 0);
	   
	   direction = reverseDirection;
	   
	   
   }

    // Called just before this Command runs the first time
	protected void initialize() {
		
		//absolute rotate
		if(this.rotateInPlace)
		{
			double distance = this.finalRotDistance;
			double start = Robot.drivetrain.getHeading();
			double circumference;
			if(distance>start) //rotate clockwise
			{
				circumference = (Math.PI*3*(distance-start)/180.0)/12.0;
				angleRot = new OneDimensionalMotionProfiling( start,  distance,  vMax,  aMax,  jMax);
			}
			else
			{
				circumference = (Math.PI*3*(start-distance)/180.0)/12.0;
				angleRot = new OneDimensionalMotionProfiling( distance,  start,  vMax,  aMax,  jMax);
				this.direction = true;
			}

			
			angleRot.S_curves();
			System.out.println(circumference);
			wheels = new OneDimensionalMotionProfiling(circumference, 8.0, 8.0, 100.0);
			wheels.S_curves();
			
			
			
			this.setPointLeft = new double[wheels.getVelArray().length];
			this.setPointRight = new double[wheels.getVelArray().length];
			
			for (int i = 0; i<setPointLeft.length; i++)
			{
				this.setPointLeft[i] = wheels.getVelArray()[i];
				this.setPointRight[i] = -wheels.getVelArray()[i];
			}
			
			
			int counterAngle = angleRot.time.length-1;
			int counterWheels = wheels.time.length-1;
			
			this.setPointHeading = new double[wheels.getTimeArray().length];
			
			//if angle array larger
			if(counterAngle >= counterWheels)
			{
				
				
				for(int i=setPointHeading.length-1; i>=0; i--)
				{
					setPointHeading[i] = angleRot.pos[counterAngle];
					counterAngle--;
				}
			}	
			else //wheels is greater than angle
			{
				for(int i=counterAngle; i>=0; i-- )
				{
					setPointHeading[counterWheels] = angleRot.pos[i];
					counterWheels--;
				}
				
				for(int i=counterWheels; i>=0; i--)
					setPointHeading[i]=angleRot.pos[0];;
				
				System.out.println("that");
			}
	 	   
			   this.headingByArray= true;
		 	   this.rotateInPlace = true;
		 	   
	 	  if(!direction)
		   {
			   this.setPointLeft = setPointLeft;
			   this.setPointRight = setPointRight;
			   this.setPointHeading = setPointHeading;
			   
		   }
		   //we want to drive the path backwards
		   // swap the left and right wheels, and negate the velocitys, also correct
		   //heading to be 180 from current position
		   else //invert heading
		   {
			   this.setPointLeft = setPointRight;
		   	   this.setPointRight = setPointLeft;
		   	   this.setPointHeading = setPointHeading;
		   	   
		   	   double[] temp = new double[this.setPointHeading.length];
		   	   int counter = this.setPointHeading.length-1;
		   	   for (int i=0; i<temp.length; i++)
		   	   {
		   		   temp[i] = this.setPointHeading[counter];
		   		   counter--;
		   	   }
		   	   
		   	   this.setPointHeading = temp;
		   }
	 	   

	 	   System.out.println("SetPointLength: " + setPointLeft.length);
		}
		
		
		
		
		
		
		
		Robot.drivetrain.leftSpeedController.reset();
		Robot.drivetrain.leftSpeedController.Enable();
		Robot.drivetrain.leftSpeedController.setSetPoint(setPointLeft);
		
		Robot.drivetrain.rightSpeedController.reset();
		Robot.drivetrain.rightSpeedController.Enable();
		Robot.drivetrain.rightSpeedController.setSetPoint(setPointRight);
    
		Robot.drivetrain.rotateDriveStraightController.reset();
		if(this.headingByArray)
			Robot.drivetrain.rotateDriveStraightController.setSetPoint(setPointHeading);
		
		Robot.drivetrain.rotateDriveStraightController.Enable();
		
		
		counter = 0;
		oldClock = Timer.getFPGATimestamp();
		
		
		Robot.drivetrain.resetPosition();

		//reset controller
		Robot.drivetrain.imu.reset();
		Robot.drivetrain.driveTrainPosController.reset();
		Robot.drivetrain.rotateDriveStraightController.reset();

		angle = Robot.drivetrain.getHeading();
		this.lastRotateOutput = 0;
		

		
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
			
			if(!this.rotateInPlace)
			{
				if (Math.abs(speedLeft)<0.15 && counter!=0)
					speedLeft = directionValue*0.15;
				
				if (Math.abs(speedRight)<0.15 && counter!=0)
					speedRight = directionValue*0.15;
			}
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
