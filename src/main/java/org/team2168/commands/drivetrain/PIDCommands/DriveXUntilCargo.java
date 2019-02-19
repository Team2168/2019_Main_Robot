package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Command;


public class DriveXUntilCargo extends Command{
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
	public DriveXUntilCargo(double distance) {
		requires(Robot.drivetrain);
		this.distanceGoal = distance;
		this.speed = RobotMap.AUTO_NORMAL_SPEED;
		this.powerShift = 1;
		this.lastRotateOutput = 0;
	}

	public DriveXUntilCargo(double distance, double speed) {
		this(distance);
		this.speed = speed;
	}

	//public DriveXDistance(double distance, double speed, double powerShift) {
	//	this(distance, speed);
	//	this.powerShift = powerShift;
	//}
	
	public DriveXUntilCargo(double distance, double speed, double error) {
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
		Robot.drivetrain.rotateDriveStraightController.reset();


		//drivetrain.resetGyro();
		endDistance = Robot.drivetrain.getAverageDistance() + distanceGoal;
		angle = Robot.drivetrain.getHeading();

//		Robot.drivetrain.rotateDriveStraightController.setpGain(RobotMap.ROTATE_POSITION_P_Drive_Straight);
//		Robot.drivetrain.rotateDriveStraightController.setiGain(RobotMap.ROTATE_POSITION_I_Drive_Straight);
//		Robot.drivetrain.rotateDriveStraightController.setdGain(RobotMap.ROTATE_POSITION_D_Drive_Straight);
		Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
		Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
		Robot.drivetrain.driveTrainPosController.setMaxNegOutput(-speed);
		Robot.drivetrain.driveTrainPosController.setMinPosOutput(0.15);
		Robot.drivetrain.driveTrainPosController.setMinNegOutput(-0.15);
		
		Robot.drivetrain.driveTrainPosController.setAcceptErrorDiff(error); //feet
		Robot.drivetrain.rotateDriveStraightController.setSetPoint(angle);
		
		
		
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
		Robot.drivetrain.rotateDriveStraightController.Enable();
	}

	protected void execute() {

		lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
		double headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput()) ;

		Robot.drivetrain.tankDrive(Robot.drivetrain.driveTrainPosController.getControlOutput()+headingCorrection, Robot.drivetrain.driveTrainPosController.getControlOutput()-headingCorrection);
		//finished = Robot.drivetrain.driveTrainPosController.isFinished();
	}



	protected void interrupted() {
		Robot.drivetrain.tankDrive(0, 0); //bypass rate limiter
	}

	protected boolean isFinished() {
		return Robot.drivetrain.driveTrainPosController.isFinished() || Robot.cargoIntakeWheels.isCargoPresent();
	}

	protected void end() {
		Robot.drivetrain.tankDrive(0, 0); //bypass rate limiter
	}

}
