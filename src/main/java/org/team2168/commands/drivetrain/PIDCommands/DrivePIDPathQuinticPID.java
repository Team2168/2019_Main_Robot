
package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivePIDPathQuinticPID extends Command
{

	private double[] setPointLeftPos;
	private double[] setPointRightPos;
	private double[] setPointLeftVel;
	private double[] setPointRightVel;
	private double[] setPointHeading;
	private double[] setPointLeftAcc;
	private double[] setPointRightAcc;

	OneDimensionalMotionProfiling motion;
	OneDimensionalMotionProfiling wheels;
	OneDimensionalMotionProfiling angleRot;

	// double vMax = 2500.0;
	// double aMax = 3000.0;
	// double jMax =30000.0;

	double vMax = 300.0;
	double aMax = 2000.0;
	double jMax = 15000.0;

	int counter;
	double ff_term = 0.0631;
	double aff_term = 0.0;
	double oldClock;
	double angle;
	double lastRotateOutput;
	boolean direction = false;
	int directionValue = 1;

	boolean limelight = false;

	private boolean headingByArray = false;
	private boolean rotateInPlace = false;
	private boolean positonGiven = false;
	double finalRotDistance;

	public DrivePIDPathQuinticPID(double distance)
	{
		this(distance, false);
	}

	public DrivePIDPathQuinticPID(double distance, boolean reverseDirection)
	{
		requires(Robot.drivetrain);
		motion = new OneDimensionalMotionProfiling(distance);
		this.setPointLeftVel = motion.getVelArray();
		this.setPointLeftPos = motion.getPosArray();
		this.setPointRightPos = motion.getPosArray();
		this.setPointRightVel = motion.getVelArray();
		this.direction = reverseDirection;
		this.positonGiven = true;
		SmartDashboard.putNumber("FF_term", 0);
		ff_term = SmartDashboard.getNumber("FF_term", 0);
	}

	public DrivePIDPathQuinticPID(double[] setPointLeftVel, double[] setPointRightVel)
	{
		requires(Robot.drivetrain);

		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;

		direction = false;

		System.out.println("SetPointLength: " + setPointLeftVel.length);
	}

	public DrivePIDPathQuinticPID(double[] setPointLeftVel, double[] setPointRightVel, double[] setPointHeading)
	{
		requires(Robot.drivetrain);

		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		this.setPointHeading = setPointHeading;

		direction = false;
		this.headingByArray = true;

		System.out.println("SetPointLength: " + setPointLeftVel.length);
	}

	public DrivePIDPathQuinticPID(double[] setPointLeftVel, double[] setPointRightVel, double[] setPointHeading,
			boolean direction)
	{
		requires(Robot.drivetrain);

		this.direction = direction;

		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		this.setPointHeading = setPointHeading;

		if (!direction)
		{
			this.setPointLeftVel = setPointLeftVel;
			this.setPointRightVel = setPointRightVel;
			this.setPointHeading = setPointHeading;

		}
		// we want to drive the path backwards
		// swap the left and right wheels, and negate the velocitys, also correct
		// heading to be 180 from current position
		else
		{
			this.setPointLeftVel = setPointRightVel;
			this.setPointRightVel = setPointLeftVel;
			this.setPointHeading = setPointHeading;

			for (int i = 0; i < this.setPointHeading.length; i++)
			{
				this.setPointHeading[i] = 180 + this.setPointHeading[i];
			}
		}

		this.headingByArray = true;

		System.out.println("SetPointLength: " + setPointLeftVel.length);
	}

	public DrivePIDPathQuinticPID(double[] setPointLeftPos, double[] setPointRightPos, double[] setPointLeftVel,
			double[] setPointRightVel, double[] setPointHeading)
	{
		requires(Robot.drivetrain);

		this.positonGiven = true;
		this.direction = false;

		this.setPointLeftPos = setPointLeftPos;
		this.setPointRightPos = setPointRightPos;
		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		this.setPointHeading = setPointHeading;

		this.headingByArray = true;

		SmartDashboard.putNumber("FF_term", this.ff_term);
		ff_term = SmartDashboard.getNumber("FF_term", this.ff_term);

		System.out.println("SetPointLength: " + setPointLeftVel.length);
	}

	public DrivePIDPathQuinticPID(double[] setPointLeftPos, double[] setPointRightPos, double[] setPointLeftVel,
			double[] setPointRightVel, double[] setPointLeftAcc,double[] setPointRightAcc, double[] setPointHeading, boolean reverse, boolean limelight)
	{
		requires(Robot.drivetrain);

		this.positonGiven = true;
		this.direction = false;

		this.limelight = limelight;

		this.setPointLeftPos = setPointLeftPos;
		this.setPointRightPos = setPointRightPos;
		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		this.setPointLeftAcc = setPointLeftAcc;
		this.setPointRightAcc= setPointRightAcc;
		this.setPointHeading = setPointHeading;

		this.headingByArray = true;

		if(reverse)
		{
			this.directionValue = -1;
		}
		else
		{
			this.directionValue = 1;
		}

		SmartDashboard.putNumber("FF_term", this.ff_term);
		ff_term = SmartDashboard.getNumber("FF_term", this.ff_term);
		SmartDashboard.putNumber("AFF_term", this.aff_term);
		aff_term = SmartDashboard.getNumber("AFF_term", this.aff_term);

		System.out.println("SetPointLength: " + setPointLeftVel.length);	
	}

	public DrivePIDPathQuinticPID(double start, double distance, double v_max, double accel_max, double j_max)
	{
		requires(Robot.drivetrain);

		this.finalRotDistance = distance;
		rotateInPlace = true;
		double circumference;
		if (distance > start) // rotate clockwise
		{
			circumference = (Math.PI * 3 * (distance - start) / 180.0) / 12.0;
			angleRot = new OneDimensionalMotionProfiling(start, distance, v_max, accel_max, j_max);
		}
		else
		{
			circumference = (Math.PI * 3 * (start - distance) / 180.0) / 12.0;
			angleRot = new OneDimensionalMotionProfiling(distance, start, v_max, accel_max, j_max);
			this.direction = true;
		}

		angleRot.S_curves();
		System.out.println(circumference);
		wheels = new OneDimensionalMotionProfiling(circumference, 8.0, 8.0, 100.0);
		wheels.S_curves();

		this.setPointLeftVel = new double[wheels.getVelArray().length];
		this.setPointRightVel = new double[wheels.getVelArray().length];

		for (int i = 0; i < setPointLeftVel.length; i++)
		{
			this.setPointLeftVel[i] = wheels.getVelArray()[i];
			this.setPointRightVel[i] = -wheels.getVelArray()[i];
		}

		int counterAngle = angleRot.time.length - 1;
		int counterWheels = wheels.time.length - 1;

		this.setPointHeading = new double[wheels.getTimeArray().length];

		// if angle array larger
		if (counterAngle >= counterWheels)
		{

			for (int i = setPointHeading.length - 1; i >= 0; i--)
			{
				setPointHeading[i] = angleRot.pos[counterAngle];
				counterAngle--;
			}
		}
		else // wheels is greater than angle
		{
			for (int i = counterAngle; i >= 0; i--)
			{
				setPointHeading[counterWheels] = angleRot.pos[i];
				counterWheels--;
			}

			for (int i = counterWheels; i >= 0; i--)
				setPointHeading[i] = angleRot.pos[0];;

			System.out.println("that");
		}

		this.headingByArray = true;
		this.rotateInPlace = true;

		if (!direction)
		{
			this.setPointLeftVel = setPointLeftVel;
			this.setPointRightVel = setPointRightVel;
			this.setPointHeading = setPointHeading;

		}
		// we want to drive the path backwards
		// swap the left and right wheels, and negate the velocitys, also correct
		// heading to be 180 from current position
		else // invert heading
		{
			this.setPointLeftVel = setPointRightVel;
			this.setPointRightVel = setPointLeftVel;
			this.setPointHeading = setPointHeading;

			double[] temp = new double[this.setPointHeading.length];
			int counter = this.setPointHeading.length - 1;
			for (int i = 0; i < temp.length; i++)
			{
				temp[i] = this.setPointHeading[counter];
				counter--;
			}

			this.setPointHeading = temp;
		}

		this.ff_term = 1.1;
		System.out.println("SetPointLength: " + setPointLeftVel.length);

	}

	public DrivePIDPathQuinticPID(double[] setPointLeftVel, double[] setPointRightVel, double ff_gain)
	{
		requires(Robot.drivetrain);
		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		ff_term = ff_gain;

		direction = false;

	}

	public DrivePIDPathQuinticPID(double[] setPointLeftVel, double[] setPointRightVel, boolean reverseDirection)
	{
		requires(Robot.drivetrain);
		this.setPointLeftVel = setPointLeftVel;
		this.setPointRightVel = setPointRightVel;
		SmartDashboard.putNumber("FF_term", 0);
		ff_term = SmartDashboard.getNumber("FF_term", 0);

		direction = reverseDirection;

	}

	// Called just before this Command runs the first time
	protected void initialize()
	{

		// absolute rotate
		if (this.rotateInPlace)
		{
			double distance = this.finalRotDistance;
			double start = Robot.drivetrain.getHeading();
			double circumference;
			if (distance > start) // rotate clockwise
			{
				circumference = (Math.PI * 3 * (distance - start) / 180.0) / 12.0;
				angleRot = new OneDimensionalMotionProfiling(start, distance, vMax, aMax, jMax);
			}
			else
			{
				circumference = (Math.PI * 3 * (start - distance) / 180.0) / 12.0;
				angleRot = new OneDimensionalMotionProfiling(distance, start, vMax, aMax, jMax);
				this.direction = true;
			}

			angleRot.S_curves();
			System.out.println(circumference);
			wheels = new OneDimensionalMotionProfiling(circumference, 8.0, 8.0, 100.0);
			wheels.S_curves();

			this.setPointLeftVel = new double[wheels.getVelArray().length];
			this.setPointRightVel = new double[wheels.getVelArray().length];

			for (int i = 0; i < setPointLeftVel.length; i++)
			{
				this.setPointLeftVel[i] = wheels.getVelArray()[i];
				this.setPointRightVel[i] = -wheels.getVelArray()[i];
			}

			int counterAngle = angleRot.time.length - 1;
			int counterWheels = wheels.time.length - 1;

			this.setPointHeading = new double[wheels.getTimeArray().length];

			// if angle array larger
			if (counterAngle >= counterWheels)
			{

				for (int i = setPointHeading.length - 1; i >= 0; i--)
				{
					setPointHeading[i] = angleRot.pos[counterAngle];
					counterAngle--;
				}
			}
			else // wheels is greater than angle
			{
				for (int i = counterAngle; i >= 0; i--)
				{
					setPointHeading[counterWheels] = angleRot.pos[i];
					counterWheels--;
				}

				for (int i = counterWheels; i >= 0; i--)
					setPointHeading[i] = angleRot.pos[0];;

				System.out.println("that");
			}

			this.headingByArray = true;
			this.rotateInPlace = true;
		}

		System.out.println("SetPointLength: " + setPointLeftVel.length);

		if (this.positonGiven)
		{
			Robot.drivetrain.leftPosController.reset();
			Robot.drivetrain.leftPosController.Enable();
			Robot.drivetrain.leftPosController.setSetPoint(this.setPointLeftPos);

			Robot.drivetrain.rightPosController.reset();
			Robot.drivetrain.rightPosController.Enable();
			Robot.drivetrain.rightPosController.setSetPoint(this.setPointRightPos);

			Robot.drivetrain.leftSpeedController.reset();
			Robot.drivetrain.leftSpeedController.Enable();
			Robot.drivetrain.leftSpeedController.setSetPoint(this.setPointLeftVel);

			Robot.drivetrain.rightSpeedController.reset();
			Robot.drivetrain.rightSpeedController.Enable();
			Robot.drivetrain.rightSpeedController.setSetPoint(this.setPointRightVel);
		}
		
		if(limelight)
		{
			Robot.drivetrain.limelightPosController.reset();

    		Robot.drivetrain.limelightPosController.setSetPoint(0);
    		Robot.drivetrain.limelightPosController.setMaxPosOutput(0.5);
    		Robot.drivetrain.limelightPosController.setMaxNegOutput(-0.5);
    		Robot.drivetrain.limelightPosController.setMinPosOutput(-0.5);
    		Robot.drivetrain.limelightPosController.setMinNegOutput(0.5);
    		Robot.drivetrain.limelightPosController.setAcceptErrorDiff(0.5);

   		 	Robot.drivetrain.limelight.setCamMode(0);
    		Robot.drivetrain.limelight.setLedMode(0);
    		Robot.drivetrain.limelight.setPipeline(0);
    		Robot.drivetrain.limelightPosController.Enable();
		}
		else
		{
			Robot.drivetrain.rotateDriveStraightController.reset();
			if (this.headingByArray)
				Robot.drivetrain.rotateDriveStraightController.setSetPoint(setPointHeading);
	
			Robot.drivetrain.rotateDriveStraightController.Enable();
		}
		

		counter = 0;
		oldClock = Timer.getFPGATimestamp();

		Robot.drivetrain.resetPosition();
		Robot.drivetrain.resetGyro();

		// reset controller
		Robot.drivetrain.imu.reset();
		Robot.drivetrain.driveTrainPosController.reset();
		Robot.drivetrain.rotateDriveStraightController.reset();

		angle = Robot.drivetrain.getHeading();
		this.lastRotateOutput = 0;

		// if true we want to reverse else we want to go forward
	// 	if (direction)
	// 		directionValue = -1;
	// 	else
	// 		directionValue = 1;
		}

	// Called repeatedly when this Command is scheduled to run

	protected void execute()
	{
		// Robot.drivetrain.tankDrive(Robot.drivetrain.leftSpeedController.getControlOutput(),
		// Robot.drivetrain.rightSpeedController.getControlOutput());

		double currTime = Timer.getFPGATimestamp();
		SmartDashboard.putNumber("Command Execution Time", (currTime - oldClock));
		oldClock = currTime;

		ff_term = SmartDashboard.getNumber("FF_term", 0);

		// lastRotateOutput =
		// Robot.drivetrain.rotateDriveStraightController.getControlOutput();
		double leftPID = 0;
		double rightPID = 0;
		double headingCorrection = 0;

		if (this.positonGiven)
		{
			// leftPID = Robot.drivetrain.leftPosController.getControlOutput()*this.directionValue;
			// rightPID = Robot.drivetrain.rightPosController.getControlOutput()*this.directionValue;
			if(limelight)
			{
				leftPID = 0;
				rightPID = 0;
			}
			else
			{
				leftPID = Robot.drivetrain.leftPosController.getControlOutput();
				rightPID = Robot.drivetrain.rightPosController.getControlOutput();
			}
		}

		if(limelight)
		{
			headingCorrection = (Robot.drivetrain.limelightPosController.getControlOutput())*-1.0;
		}
		else
		{
			headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());
		}

		if (counter < setPointLeftVel.length)
		{
			double speedLeft = (ff_term * setPointLeftVel[counter]) / (Robot.pdp.getBatteryVoltage());
			double speedRight = (ff_term * setPointRightVel[counter]) / (Robot.pdp.getBatteryVoltage());
			double accelLeft = (aff_term * setPointLeftAcc[counter]) / (Robot.pdp.getBatteryVoltage());//0
			double accelRight = (aff_term * setPointRightAcc[counter]) / (Robot.pdp.getBatteryVoltage());//0

			if (!this.rotateInPlace)
			{
				if (Math.abs(speedLeft) < 0.15 && counter != 0)
					speedLeft = directionValue * 0.15;

				if (Math.abs(speedRight) < 0.15 && counter != 0)
					speedRight = directionValue * 0.15;
			}
			Robot.drivetrain.tankDrive(speedLeft + headingCorrection + leftPID +accelLeft,
					speedRight - headingCorrection + rightPID + accelRight);
			// Robot.drivetrain.tankDrive(speedLeft+leftPID,speedRight+rightPID);
			counter++;

			SmartDashboard.putNumber("speedLeft", speedLeft);
			SmartDashboard.putNumber("speedRight", speedRight);
			SmartDashboard.putNumber("headingCorrection", headingCorrection);
			SmartDashboard.putNumber("leftPID", leftPID);
			SmartDashboard.putNumber("rightPID", rightPID);
			SmartDashboard.putNumber("accelLeft", accelLeft);
			SmartDashboard.putNumber("accelRight", accelRight);
			SmartDashboard.putNumber("FF_term", ff_term);
			SmartDashboard.putNumber("leftSum", speedLeft+headingCorrection+leftPID);
			SmartDashboard.putNumber("rightSum", speedRight+headingCorrection+rightPID);
		}
		else
		{
			Robot.drivetrain.tankDrive(0.0, 0.0);
		}



	}

	// Make this return true when this Command no longer needs to run execute()

	protected boolean isFinished()
	{

		return (counter >= setPointLeftVel.length);
	}

	// Called once after isFinished returns true

	protected void end()
	{
		// Robot.drivetrain.leftPosController.Pause();
		// Robot.drivetrain.rightPosController.Pause();
		// Robot.drivetrain.leftSpeedController.Pause();
		// Robot.drivetrain.rightSpeedController.Pause();
		// Robot.drivetrain.rotateDriveStraightController.Pause();
		//// Robot.drivetrain.rotateDriveStraightController.reset();
		//// Robot.drivetrain.leftPosController.reset();
		//// Robot.drivetrain.rightPosController.reset();
		Robot.drivetrain.tankDrive(0, 0);
		Robot.drivetrain.limelightPosController.Pause();
		Robot.drivetrain.limelight.setCamMode(1);
		Robot.drivetrain.limelight.setLedMode(1);
		Robot.drivetrain.limelight.setPipeline(7);

	}

	// delete me
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run

	protected void interrupted()
	{
		end();
	}
}
