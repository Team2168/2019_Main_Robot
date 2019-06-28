package org.team2168.commands.drivetrain;


import org.team2168.OI;
import org.team2168.Robot;
import org.team2168.RobotMap;


import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveWithJoystick extends Command {

	int ctrlStyle;
	/**
	 * Controller Styles 0 = Tank Drive (Default) 1 = Gun Style 2 = Arcade Drive 3 =
	 * GTA
	 * 
	 * @param inputStyle
	 */

	private double distanceGoal;
	private double speed;
	private double endDistance;
	private boolean finished;
	private double angle;
	private double error = 0.1;

	private int intCounter = 0;
	private double powerShift;

	double rightSpeed = 0;
	double leftSpeed = 0;

	static final double DIST_ERROR_TOLERANCE_INCH = 1;
	static final double TURN_ERROR_TOLERANCE_DEG = 1;

	private int climbCounter = 0;
	private int climbCounterReverse = 0;

	double lastRotateOutput;
	DisengageDrivetrain disengageDrivetrainCommand;

	public DriveWithJoystick(int inputStyle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drivetrain);
		ctrlStyle = inputStyle;
		// TODO not sure where distanceGoal is used but for our tests we used a value of
		// 1
		this.distanceGoal = 1;
		this.speed = RobotMap.AUTO_NORMAL_SPEED;
		this.powerShift = 1;
		this.lastRotateOutput = 0;
		
		SmartDashboard.putNumber("Recordnumber", 0);
	}

	// Called just before this Command runs the first time
	
	
	protected void initialize() {
		
		
		intCounter = 0;
		ctrlStyle = Robot.getControlStyleInt();
		switch (ctrlStyle) {
		/**
		 * Initialize driveStraightController for Gun style
		 */
		case 1:
			finished = false;
			Robot.drivetrain.getInstance();
		
			// reset controller
				Robot.drivetrain.resetPosition();	
				Robot.drivetrain.imu.reset();
				Robot.drivetrain.driveTrainPosController.reset();
				Robot.drivetrain.rotateDriveStraightController.reset();

			// drivetrain.resetGyro();
			endDistance = Robot.drivetrain.getAverageDistance() + distanceGoal;
			angle = Robot.drivetrain.getHeading();

			// Robot.drivetrain.rotateDriveStraightController.setpGain(RobotMap.ROTATE_POSITION_P_Drive_Straight);
			// Robot.drivetrain.rotateDriveStraightController.setiGain(RobotMap.ROTATE_POSITION_I_Drive_Straight);
			// Robot.drivetrain.rotateDriveStraightController.setdGain(RobotMap.ROTATE_POSITION_D_Drive_Straight);
			Robot.drivetrain.driveTrainPosController.setSetPoint(endDistance);
			Robot.drivetrain.driveTrainPosController.setMaxPosOutput(speed);
			Robot.drivetrain.driveTrainPosController.setMinPosOutput(-speed);
			Robot.drivetrain.driveTrainPosController.setAcceptErrorDiff(error); // feet
			Robot.drivetrain.rotateDriveStraightController.setSetPoint(angle);

			// //This code helps offset uneven left/right gearbox power, like a feedworward
			// term
			// //modify speeds based on power shift, - means put more power to left side, +
			// means put more power to right side
			// //this power shift helps accommodate for unequal power in drivetrains
			//
			// if (powerShift > 1) //reduce left speed so right power is increased
			// {
			// rightSpeed = speed;
			// leftSpeed = speed - speed*Math.abs(powerShift%1);
			// }
			//
			// else if (powerShift < 1) //reduce right speed so left if increased
			// {
			// rightSpeed = speed - speed*Math.abs(powerShift%1);
			// leftSpeed = speed;
			// }
			//
			// Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.gyroSPI.getAngleDeg());

			Robot.drivetrain.driveTrainPosController.Enable();
			//Robot.drivetrain.rotateDriveStraightController.Enable();

			System.out.println("Initialize case ran");
		default:

			break;
		}
	}

	/**
	 * Gets the joystick positions from OI and sends them to the drivetrain
	 * subsystem.
	 * 
	 * @author Krystina
	 */
	protected void execute() {
		double headingCorrection = 0.0;
		ctrlStyle = Robot.getControlStyleInt();

		switch (ctrlStyle) {
		/**
		 * Tank Drive
		 */
		case 0:
			Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getLeftStickRaw_Y());
			Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getRightStickRaw_Y());
			break;

		/**
		 * Gun Style Controller
		 */
		// X Values
		// full in: -0.516
		// nothing: 0.354 & 0.342
		// full out: 0.622
		case 1:
			{
			lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
			headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());

			//
			double minStingerVoltage = 4.0/Robot.pdp.getBatteryVoltage();

			if(OI.getInstance().driverJoystick.isPressedButtonLeftBumper())
			{
				Robot.drivetrain.tankDrive(minStingerVoltage, 0.0);
				System.out.println("Left Speed:" + speed);
			}
			if(OI.getInstance().driverJoystick.isPressedButtonRightBumper())
			{
				Robot.drivetrain.tankDrive(0.0, minStingerVoltage);
				System.out.println("Right Speed:" + speed);
			}
			if(Robot.isClimbEnabled)
			{ 
				if (climbCounter < 25) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
				{
					double voltage = 1.0;
					double speed = voltage/Robot.pdp.getBatteryVoltage();
					Robot.drivetrain.tankDrive(speed,speed);
					climbCounter++;
				}
				else
					Robot.drivetrain.tankDrive(Robot.oi.getGunStyleYValue(), Robot.oi.getGunStyleYValue());
			}
			else if ((Robot.oi.driverJoystick.getLeftStickRaw_X() < 0.1) && (Robot.oi.driverJoystick.getLeftStickRaw_X() > -0.1))
			{
				Robot.drivetrain.tankDrive(Robot.oi.getGunStyleYValue(), Robot.oi.getGunStyleYValue());	
				climbCounter = 0;
				climbCounterReverse = 0;
			} 
			else {
				Robot.drivetrain.tankDrive(
						(Robot.oi.getGunStyleYValue()) + Robot.oi.driverJoystick.getLeftStickRaw_X(),
						(Robot.oi.getGunStyleYValue()) - Robot.oi.driverJoystick.getLeftStickRaw_X());
				Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
				climbCounter = 0;
				climbCounterReverse = 0;
						
			}
			
			
		}
			break;

		/**
		 * Arcade Drive
		 */
		case 2:
			Robot.drivetrain.driveLeft(
					Robot.oi.driverJoystick.getLeftStickRaw_Y() + Robot.oi.driverJoystick.getRightStickRaw_X());
			Robot.drivetrain.driveRight(
					Robot.oi.driverJoystick.getLeftStickRaw_Y() - Robot.oi.driverJoystick.getRightStickRaw_X());
			break;
		/**
		 * GTA Drive
		 */
		case 3:
			double fwdSpeed = Robot.oi.driverJoystick.getRightTriggerAxisRaw();
			double revSpeed = Robot.oi.driverJoystick.getLeftTriggerAxisRaw();
			double speed = fwdSpeed - revSpeed;
			double rotation = Robot.oi.driverJoystick.getRightStickRaw_X();

			// Adjusts angle while moving
			if (speed != 0 && rotation != 0) {
				Robot.drivetrain.driveLeft(rotation * speed);
				Robot.drivetrain.driveRight(-rotation * speed);
			}
			// Allows Robot to spin in place without needing to press in triggers
			else if (speed == 0 && rotation != 0) {
				Robot.drivetrain.driveLeft(rotation);
				Robot.drivetrain.driveRight(-rotation);
			}
			// Allows Robot to drive straight
			else if (speed != 0 && rotation == 0) {
				Robot.drivetrain.driveLeft(speed);
				Robot.drivetrain.driveRight(speed);
			}
			break;

			/**
			 * New Gun Style Controller
			 */
			case 4:
				lastRotateOutput = Robot.drivetrain.rotateDriveStraightController.getControlOutput();
				headingCorrection = (Robot.drivetrain.rotateDriveStraightController.getControlOutput());
			//
			double minStingerVoltage = 4.0/Robot.pdp.getBatteryVoltage();

			if(OI.getInstance().driverJoystick.isPressedButtonLeftBumper())
			{
				Robot.drivetrain.tankDrive(0.0, minStingerVoltage);
				System.out.println("Right speed:" + minStingerVoltage);
				return;
			}

			if(OI.getInstance().driverJoystick.isPressedButtonRightBumper())
			{
				Robot.drivetrain.tankDrive(minStingerVoltage, 0.0);
				System.out.println("Left Speed:" + minStingerVoltage);
				return;
			}

			if(Robot.isClimbEnabled)
			{ 
				if( disengageDrivetrainCommand == null)
					disengageDrivetrainCommand = new DisengageDrivetrain();

				
				if (climbCounter < 25) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
				{
					double voltage = 2.0;
					double minspeed = voltage/Robot.pdp.getBatteryVoltage();
					Robot.drivetrain.tankDrive(minspeed,minspeed);
					System.out.println("Driving stinger slow");
					climbCounter++;
				}
				else
				{
					if(-Robot.oi.driverJoystick.getY(Hand.kLeft)>0.1)
						Robot.drivetrain.tankDrive(-Robot.oi.driverJoystick.getY(Hand.kLeft), -Robot.oi.driverJoystick.getY(Hand.kLeft));
					else //driving dt in reverse, lets not do that
					{
						if (climbCounterReverse < 15) //auto drive drivetrain for a small time 0.5 seconds 7*0.02 to help engage
						{
							if(!disengageDrivetrainCommand.isRunning())
								disengageDrivetrainCommand.start();

							double voltage = 1.0;
							double minspeed = voltage/Robot.pdp.getBatteryVoltage();
							Robot.drivetrain.tankDrive(minspeed,minspeed);
							System.out.println("Driving stinger slow");
							climbCounterReverse++;
						}
						else if (!Robot.isClimbEnabledLevel2)
						{
							if(Robot.drivetrain.getLeftStingerPosition()>0.0)
							{
								//if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								Robot.drivetrain.driveLeft(-Robot.oi.driverJoystick.getY(Hand.kLeft));
							}
							else
							{
								Robot.drivetrain.driveLeft(0.0);
							}

							if(Robot.drivetrain.getRightStingerPosition()>0.0)
							{
								//if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								Robot.drivetrain.driveRight(-Robot.oi.driverJoystick.getY(Hand.kLeft));
							}
							else
							{
								Robot.drivetrain.driveRight(0.0);
							}
						}
						else
						{
							if(Robot.drivetrain.getLeftStingerPosition()>0.0)
							{
								if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								{
								Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getY(Hand.kLeft));
								}
							}
							else
							{
								Robot.drivetrain.driveLeft(0.0);
							}

							if(Robot.drivetrain.getRightStingerPosition()>0.0)
							{
								if(Robot.oi.driverJoystick.getY(Hand.kLeft)>0.0)
								{
								Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getY(Hand.kLeft));
								}
							}
							else
							{
								Robot.drivetrain.driveRight(0.0);
							}
						}
					}
					}
				}
			else if (Math.abs(Robot.oi.driverJoystick.getX(Hand.kLeft)) < 0.1) 
			{
					climbCounter = 0;
					climbCounterReverse = 0;
					//Drive straight
					if(Robot.drivetrain.limelightPosController.isEnabled())
					{
						Robot.drivetrain.tankDrive(
						-Robot.oi.driverJoystick.getY(Hand.kLeft) - Robot.drivetrain.limelightPosController.getControlOutput(),
						-Robot.oi.driverJoystick.getY(Hand.kLeft) + Robot.drivetrain.limelightPosController.getControlOutput());
					}
					else {
						Robot.drivetrain.tankDrive(-Robot.oi.driverJoystick.getY(Hand.kLeft),
							-Robot.oi.driverJoystick.getY(Hand.kLeft));
					}	
				} else {
					//Arcade drive
					if(Robot.drivetrain.limelightPosController.isEnabled())
					{
						Robot.drivetrain.tankDrive(
							Robot.oi.getGunStyleYValue() + Robot.oi.driverJoystick.getX(Hand.kLeft) - Robot.drivetrain.limelightPosController.getControlOutput(),
							Robot.oi.getGunStyleYValue() - Robot.oi.driverJoystick.getX(Hand.kLeft) + Robot.drivetrain.limelightPosController.getControlOutput());
						Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
					}
					else {
						Robot.drivetrain.tankDrive(
								Robot.oi.getGunStyleYValue() + Robot.oi.driverJoystick.getX(Hand.kLeft),
								Robot.oi.getGunStyleYValue() - Robot.oi.driverJoystick.getX(Hand.kLeft));
						Robot.drivetrain.rotateDriveStraightController.setSetPoint(Robot.drivetrain.getHeading());
					}
				}
				break;
		/**
		 * Defaults to Tank Drive
		 */
		default:
			Robot.drivetrain.driveLeft(Robot.oi.driverJoystick.getLeftStickRaw_Y());
			Robot.drivetrain.driveRight(Robot.oi.driverJoystick.getRightStickRaw_Y());
			break;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drivetrain.tankDrive(0.0, 0.0);
	}
	

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		
		end();
	}
	
}
