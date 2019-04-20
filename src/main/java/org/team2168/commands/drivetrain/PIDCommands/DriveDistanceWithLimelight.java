package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;


public class DriveDistanceWithLimelight extends Command{
	private double distanceGoal;
	private double speed;
	private double endDistance;
	private boolean finished;
	private double angle;
	private double error = 0.3;
	
	private double powerShift;

	double rightSpeed = 0;
	double leftSpeed = 0;

	static final double DIST_ERROR_TOLERANCE_INCH = 1;
	static final double TURN_ERROR_TOLERANCE_DEG =1;

	double lastRotateOutput;

	/**
	 * Move the drivetrain forward the specified distance.
	 * @param distance in feet
	 */
	public DriveDistanceWithLimelight(double distance) {
		requires(Robot.drivetrain);
		this.distanceGoal = distance;
		this.speed = RobotMap.AUTO_NORMAL_SPEED;
		this.powerShift = 1;
		this.lastRotateOutput = 0;
	}

	public DriveDistanceWithLimelight(double distance, double speed) {
		this(distance);
		this.speed = speed;
	}

	//public DriveXDistance(double distance, double speed, double powerShift) {
	//	this(distance, speed);
	//	this.powerShift = powerShift;
	//}
	
	public DriveDistanceWithLimelight(double distance, double speed, double error) {
		this(distance, speed);
		this.error = error;
	}

	protected void initialize() {
		finished = false;
		Robot.drivetrain.tankDrive(0, 0);
		Robot.drivetrain.resetPosition();

		//reset controller
		Robot.drivetrain.imu.reset();
		Robot.drivetrain.driveTrainPosController.reset();
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

		//drivetrain.resetGyro();
		endDistance = Robot.drivetrain.getAverageDistance() + distanceGoal;
		//angle = Robot.drivetrain.getHeading();

//		Robot.drivetrain.rotateDriveStraightController.setpGain(RobotMap.ROTATE_POSITION_P_Drive_Straight);
//		Robot.drivetrain.rotateDriveStraightController.setiGain(RobotMap.ROTATE_POSITION_I_Drive_Straight);
//		Robot.drivetrain.rotateDriveStraightController.setdGain(RobotMap.ROTATE_POSITION_D_Drive_Straight);
		Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
		Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
		Robot.drivetrain.driveTrainPosController.setMaxNegOutput(-speed);
		Robot.drivetrain.driveTrainPosController.setMinPosOutput(0.15);
		Robot.drivetrain.driveTrainPosController.setMinNegOutput(-0.15);
		
		Robot.drivetrain.driveTrainPosController.setAcceptErrorDiff(error); //feet
		//Robot.drivetrain.rotateDriveStraightController.setSetPoint(angle);
		
		
		
		// 		//This code helps offset uneven left/right gearbox power, like a feedworward term
		//		//modify speeds based on power shift, - means put more power to left side, + means put more power to right side
		//		//this power shift helps accommodate for unequal power in drivetrains
		//
		//		if (powerShift > 1) //reduce left speed so right power is increased
		//		{
		//			rightSpeed = speed;
		//			leftSpeed = speed - speed*Math.abs(powerShift%1);
		//		}
		//
		//		else if (powerShift < 1) //reduce right speed so left if increased
		//		{
		//			rightSpeed = speed - speed*Math.abs(powerShift%1);
		//			leftSpeed = speed;
		//		}
		//
		//		Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.gyroSPI.getAngleDeg());

		Robot.drivetrain.driveTrainPosController.Enable();
		Robot.drivetrain.limelightPosController.Enable();
	}

	protected void execute() {

		//Set Min as minimum voltage to drive mechanims based on emperical measurements. Should account for battery dipping. Calculate Motor controller %
		Robot.drivetrain.driveTrainPosController.setMinPosOutput(RobotMap.DRIVE_TRAIN_MIN_FWD_VOLTAGE/Robot.pdp.getBatteryVoltage());
		Robot.drivetrain.driveTrainPosController.setMinNegOutput(-RobotMap.DRIVE_TRAIN_MIN_FWD_VOLTAGE/Robot.pdp.getBatteryVoltage());
		
		double headingCorrection = -1.0*(Robot.drivetrain.limelightPosController.getControlOutput()) ;

		Robot.drivetrain.tankDrive(Robot.drivetrain.driveTrainPosController.getControlOutput()+headingCorrection, Robot.drivetrain.driveTrainPosController.getControlOutput()-headingCorrection);
		//finished = Robot.drivetrain.driveTrainPosController.isFinished();
	}



	protected void interrupted() {
		Robot.drivetrain.tankDrive(0, 0); //bypass rate limiter
	}

	protected boolean isFinished() {
		return Robot.drivetrain.driveTrainPosController.isFinished();
	}

	protected void end() {
		Robot.drivetrain.tankDrive(0, 0); //bypass rate limiter
		Robot.drivetrain.limelightPosController.Pause();
		Robot.drivetrain.limelight.setCamMode(1);
		Robot.drivetrain.limelight.setLedMode(1);
		Robot.drivetrain.limelight.setPipeline(7);
	}

}
