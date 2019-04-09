package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.commands.LEDs.LiftLoweringPattern;
import org.team2168.commands.LEDs.LiftRaisingPattern;
import org.team2168.commands.hatchProbePivot.interlock.MovePivotToMBPosition;
import org.team2168.commands.lift.DriveLiftWithJoysticks;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToSafePositionForLift;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subystem class for the lift
 * 
 * 
 **/
 
public class Lift extends Subsystem {
	
	private static TalonSRX liftMotor1;
	private static VictorSPX liftMotor2;


	//private DoubleSolenoid liftBrake;
	private static AveragePotentiometer liftPot;
	private static DigitalInput liftFullyUp; // hall effect sensors
	private static DigitalInput liftFullyDown; // ^^^^^^^^^^^

	double liftPotMax;
	double liftPotMin;

	public PIDPosition liftPOTController;
	TCPSocketSender TCPLiftPOTController;

	public volatile double liftMotor1Voltage;
	public volatile double liftMotor2Voltage;


	private boolean liftMotor1Fault = false;
	private boolean liftMotor2Fault = false;


	private boolean liftMotor1HighCurrent = false;
	private boolean liftMotor2HighCurrent = false;


	private boolean liftMotor1HighThenZeroCurrent = false;
	private boolean liftMotor2HighThenZeroCurrent = false;


	private boolean isLiftMotor1BreakerTrip = false;
	private boolean isLiftMotor2BreakerTrip = false;

	private static Lift instance = null;

	private int timeCounter = 0;

	private boolean isSensorValid = true;

	//interlock commands ()
	MoveMonkeyBarToSafePositionForLift moveMonkeyBarToSafePositionForLift;
	MovePivotToMBPosition movePivotToMBPosition;


	public double liftSpeedForLEDs;

	/**
	 * Default constructor for the lift
	 */
	private Lift() {
		liftMotor1 = new TalonSRX(RobotMap.LIFT_MOTOR_1_PDP);
		liftMotor2 = new VictorSPX(RobotMap.LIFT_MOTOR_2_PDP);
		
		//liftBrake = new DoubleSolenoid(RobotMap.PCM_CAN_ID_BELLYPAN, RobotMap.LIFT_BRAKE_ENGAGE_PCM, RobotMap.LIFT_BRAKE_DISENGAGE_PCM);
		liftFullyUp = new DigitalInput(RobotMap.LIFT_FULLY_UP_LIMIT);
		liftFullyDown = new DigitalInput(RobotMap.LIFT_FULLY_DOWN_LIMIT);

			

    if (Robot.isPracticeRobot()) 
    {
      liftPot = new AveragePotentiometer(RobotMap.LIFT_POSITION_POT, 
        RobotMap.LIFT_POT_VOLTAGE_0_PBOT,
        RobotMap.LIFT_POT_0_HEIGHT_INCHES_PBOT, 
        RobotMap.LIFT_POT_VOLTAGE_MAX_PBOT,
        RobotMap.LIFT_POT_MAX_HEIGHT_INCHES_PBOT, 
        RobotMap.LIFT_AVG_ENCODER_VAL);
      
      liftPotMax = RobotMap.LIFT_POT_VOLTAGE_MAX_PBOT;
			liftPotMin = RobotMap.LIFT_POT_VOLTAGE_0_PBOT;
    } 
    else 
    {
      liftPot = new AveragePotentiometer(liftMotor1, 
        RobotMap.LIFT_POT_VOLTAGE_0,
        RobotMap.LIFT_POT_0_HEIGHT_INCHES, 
        RobotMap.LIFT_POT_VOLTAGE_MAX,
        RobotMap.LIFT_POT_MAX_HEIGHT_INCHES, 
        RobotMap.LIFT_AVG_ENCODER_VAL);
      
      liftPotMax = RobotMap.LIFT_POT_VOLTAGE_MAX;
			liftPotMin = RobotMap.LIFT_POT_VOLTAGE_0;
		}

    liftPOTController = new PIDPosition("LiftPosController", 
      RobotMap.LIFT_P, 
      RobotMap.LIFT_I, 
      RobotMap.LIFT_D,
      liftPot, 
      RobotMap.LIFT_PID_PERIOD);

		liftPOTController.setSIZE(RobotMap.LIFT_PID_ARRAY_SIZE);

		liftPOTController.startThread();

		TCPLiftPOTController = new TCPSocketSender(RobotMap.TCP_SERVER_LIFT_POT_CONTROLLER, liftPOTController);
		TCPLiftPOTController.start();

		ConsolePrinter.putNumber("Lift Joystick value", () -> {return Robot.oi.getLiftJoystickValue();}, true, true);
		ConsolePrinter.putNumber("Lift motor 1 voltage", () -> {return liftMotor1Voltage;}, true, true);
		ConsolePrinter.putNumber("Lift motor 2 voltage", () -> {return liftMotor2Voltage;}, true, true);

		ConsolePrinter.putNumber("Lift Motor 1 Current ", () -> {return Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP);}, true, true);
		ConsolePrinter.putNumber("Lift Motor 2 Current ", () -> {return Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP);}, true, true);


		ConsolePrinter.putBoolean("Is Lift Fully Up", () -> {return Robot.lift.isLiftFullyUp();}, true, false);
	  	ConsolePrinter.putBoolean("Is Lift Fully Down", () -> {return Robot.lift.isLiftFullyDown();}, true, false);
		ConsolePrinter.putNumber("Lift Raw Pot", () -> {return getRawPot();}, true, false);
		ConsolePrinter.putNumber("Lift Pot Inches", () -> {return getPotPos();}, true, false);
		ConsolePrinter.putNumber("Lift Pot Rate", () -> {return getPotRate();}, true, false);
		ConsolePrinter.putBoolean("Lift Is Sensor Valid", () -> {return isSensorValid();}, true, false);

		ConsolePrinter.putBoolean("Lift Motor1_FAULT", () -> {return liftMotor1Fault;}, true, false);
		ConsolePrinter.putBoolean("Lift Motor2_FAULT", () -> {return liftMotor2Fault;}, true, false);


		ConsolePrinter.putBoolean("Lift Motor1_Breaker_Trip", () -> {return isLiftMotor1BreakerTrip;}, true, false);
		ConsolePrinter.putBoolean("Lift Motor2_Breaker_Trip", () -> {return isLiftMotor2BreakerTrip;}, true, false);

	}

	/**
	 * Singleton constructor of the lift
	 * 
	 */

  	public static Lift getInstance()
	{
		if (instance == null)
			instance = new Lift();
		return instance;
	}

	/**
	 * 
	 * @return pot position in volts
	 */
	public double getRawPot()
	{
		return liftPot.getRawPos();
	}

	/**
	 * 
	 * @return pot position in inches
	 */
	public double getPotPos()
	{
		return liftPot.getPos();
	}

		/**
	 * 
	 * @return pot position in inches
	 */
	public double getPotRate()
	{
		return liftPot.getRate();
	}

	/**
	 * Checks to see if arm is fully up
	 * 
	 * @return true if pressed, false if not
	 */
	public boolean isLiftFullyUp()
	{
		return !liftFullyUp.get();
	}

	/**
	 * Checks to see if arm is fully down
	 * 
	 * @return true if pressed, false if not
	 */
	public boolean isLiftFullyDown()
	{
		return !liftFullyDown.get();
	}

	/**
	 * Drives the first Lift motor at a speed from -1 to 1 where 1 is forward and
	 * negative 1 is backwards
	 * 
	 * @param speed
	 */
	private void driveLiftMotor1(double speed)
	{
		if (RobotMap.LIFT_MOTOR1_REVERSE)
			speed = -speed;
		liftMotor1.set(ControlMode.PercentOutput,speed);
		liftMotor1Voltage = Robot.pdp.getBatteryVoltage() * speed;
	}

	/**
	 * Drives the second Lift motor at a speed from -1 to 1 where 1 is forward and
	 * negative 1 is backwards
	 * 
	 * @param speed
	 */
	private void driveLiftMotor2(double speed)
	{
		if (RobotMap.LIFT_MOTOR2_REVERSE)
			speed = -speed;
		liftMotor2.set(ControlMode.PercentOutput,speed);
		liftMotor2Voltage = Robot.pdp.getBatteryVoltage() * speed;
	}

	/**
	 * Drives all lift Motors at a speed from -1 to 1 where 1 is up and negative 1
	 * is down, and ratchet is not engaged
	 * 
	 * @param speed is +1 up and -1 down
	 */
	public void driveAllMotors(double speed)
	{
		this.liftSpeedForLEDs = 0;
		if( moveMonkeyBarToSafePositionForLift == null)
			moveMonkeyBarToSafePositionForLift = new MoveMonkeyBarToSafePositionForLift();
		if( movePivotToMBPosition == null)
			movePivotToMBPosition = new MovePivotToMBPosition();	
		
		// lift is stalling
		if ((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) > RobotMap.STALL_CURRENT_LIMIT)
				|| (Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > RobotMap.STALL_CURRENT_LIMIT))
		{

				driveLiftMotor1(0.0);
				driveLiftMotor2(0.0);
		

			System.out.println("We stalling Lift");
		}
		else
		{
			timeCounter = 0;

			if (RobotMap.ENABLE_LIFT_POT_SAFETY) //use pot value as additional sensor check
			{
				//If we are commanding the lift, only allow it to go up if not at hard stop or the pot is not at max value
				//Similarly only allow it to go down if we are not at the lower limit or the pot is not at its lowest limit
				
				boolean drivingUp = speed > RobotMap.LIFT_MIN_SPEED;
				boolean drivingDown = speed < -RobotMap.LIFT_MIN_SPEED;

				if (( drivingUp && !isLiftFullyUp() && !liftPot.isAtLowerLimit() )
						|| (drivingDown && !isLiftFullyDown() && !liftPot.isAtUpperLimit()))
				{

					// //Driving up but need to prevent crashing into lift support 
					if(RobotMap.LIFT_ENABLE_INTERLOCKS && drivingUp && Robot.hatchProbePivot.isOnMonkeyBarSide() && liftPot.getPos() >= RobotMap.LIFT_POT_CROSS_BAR_HEIGHT)
					{
						//Stop Moving lift
						driveLiftMotor1(0.0);
						driveLiftMotor2(0.0);
						System.out.println("Interlock:LIFT: Lift trying to drive into Lift Cross Support");
						return;
					}

					//we trying to drive up but pivot is not fully on one side on PivotSide. need to move pivot
					if(RobotMap.LIFT_ENABLE_INTERLOCKS && drivingUp && Robot.hatchProbePivot.isOnMonkeyBarSide() &&  !Robot.hatchProbePivot.isSafeToMoveLiftUp() && !movePivotToMBPosition.isRunning())
					{
						movePivotToMBPosition.start();
						System.out.println("Interlock:LIFT: Lift trying to drive up while pivot not safe on MB side. Moving Pivot to safe angle");
						return;
					}
					else if(movePivotToMBPosition.isRunning())
					{
						//Stop Moving lift and wait for pivot to move
						driveLiftMotor1(0.0);
						driveLiftMotor2(0.0);
						System.out.println("Interlock:LIFT: Lift trying to drive while hatch pivot safe down waiting for pivot to move.");
						return;
					}

					//we trying to drive up but MonkeyBar in way
					if(RobotMap.LIFT_ENABLE_INTERLOCKS && drivingUp && Robot.hatchProbePivot.isOnMonkeyBarSide() &&  !Robot.monkeyBarPivot.isSafeLiftPosition() && !moveMonkeyBarToSafePositionForLift.isRunning())
					{
						moveMonkeyBarToSafePositionForLift.start();
						System.out.println("Interlock:LIFT: Lift trying to drive up while pivot not safe on MB side. Moving Pivot to safe angle");
						//movedMonkeyBar = true;
						return;
					}
					else if (moveMonkeyBarToSafePositionForLift.isRunning())
					{
						//Stop Moving lift and wait for pivot to move
						driveLiftMotor1(0.0);
						driveLiftMotor2(0.0);
						System.out.println("Interlock:LIFT: Lift trying to drive while MB pivot not down waiting for pivot to move.");
						return;
				
					}

					//if we got here then we are driving lift
					driveLiftMotor1(speed);
					driveLiftMotor2(speed);
					this.liftSpeedForLEDs = speed;

					if(Math.abs(liftPot.getRate()) < 1.0)
						isSensorValid = false;
					else
						isSensorValid = true;

					
				}
				else if(RobotMap.LIFT_ENABLE_HEIGHT_HOLD)
				{
					double holdingSpeed = RobotMap.LIFT_HOLDING_VOLTAGE/Robot.pdp.getBatteryVoltage();
					if(getPotPos() <= RobotMap.LIFT_ZERO_BELOW_THIS_HEIGHT)
					{
						driveLiftMotor1(-holdingSpeed);//+0.01
						driveLiftMotor2(-holdingSpeed);
					//  System.out.println("Da lift is holding down");
					}
					else
					{
						driveLiftMotor1(holdingSpeed);
						driveLiftMotor2(holdingSpeed);
					 // System.out.println("Da lift is holding up");
					}
				}
				else
				{
					driveLiftMotor1(0.0);
					driveLiftMotor2(0.0);
				}
			}
			else
			{
				if ((speed > RobotMap.LIFT_MIN_SPEED && !isLiftFullyUp())
						|| ((speed < -RobotMap.LIFT_MIN_SPEED) && !isLiftFullyDown()))
				{
					driveLiftMotor1(speed);
					driveLiftMotor2(speed);
					this.liftSpeedForLEDs = speed;
				}
				else
				{
					driveLiftMotor1(0.0);
					driveLiftMotor2(0.0);
				}

				
			}
		}

		isLiftMotor1Failure();
		isLiftMotor2Failure();

		isLiftMotor1BreakerTrip();
		isLiftMotor2BreakerTrip();
	}


	/**
	 * Drives all lift Motors at a speed from -1 to 1 where 1 is up and negative 1
	 * is down, and ratchet is not engaged
	 * 
	 * @param speed is +1 up and -1 down
	 */
	public void driveAllMotorsNoInterlocks(double speed)
	{

		double stallLimit = 35;
		// lift is stalling
		if ((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) > stallLimit)
				|| (Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > stallLimit))
		{
			// enableBrake();
			timeCounter++;

			// wait for brake to actuate then stop motors
			if (timeCounter >= 1 / .02)
			{
				driveLiftMotor1(0.0);
				driveLiftMotor2(0.0);
			}
		}
		else
		{
			timeCounter = 0;

			if (RobotMap.ENABLE_LIFT_POT_SAFETY) //use pot value as additional sensor check
			{

				// TODO: Implement interlocks for

				if ((speed > RobotMap.LIFT_MIN_SPEED && !isLiftFullyUp()
						&& !liftPot.isAtUpperLimit())
						|| ((speed < -RobotMap.LIFT_MIN_SPEED) && !isLiftFullyDown()))
				{

					//todo: Add lift interlocks
					// if (Robot.isAutoMode())
					// {
					// 	if (Robot.lift.getPotPos() > 70 && (speed > 0))
					// 		Robot.intakePivotPiston.retracPivotPiston();
					// }

					// if (!Robot.isAutoMode() && !Robot.flipperyFloopyFlupy.getHardStopStatus())
					// {
					// 	if (Robot.lift.getPotPos() > 15 && (speed > 0))
					// 	{
					// 		Robot.intakePivotPiston.retracPivotPiston();
					// 	}
					// }
					// disableBrake();
					driveLiftMotor1(speed);
					driveLiftMotor2(speed);

					// if(Robot.lift.getPotPos() > 0 && Robot.lift.getPotPos() < 30.0)
					// Robot.i2c.write(8, 8);
					// if(Robot.lift.getPotPos() > 30.0 && Robot.lift.getPotPos() < 60.0)
					// Robot.i2c.write(8, 3);
					// if(Robot.lift.getPotPos() > 60.0 && Robot.lift.getPotPos() < 82.5)
					// Robot.i2c.write(8, 11);

				}
				else
				{
					// enableBrake();
					driveLiftMotor1(0.0);
					driveLiftMotor2(0.0);

				}
			}
			else
			{
				if ((speed > RobotMap.LIFT_MIN_SPEED && !isLiftFullyUp())
						|| ((speed < -RobotMap.LIFT_MIN_SPEED) && !isLiftFullyDown()))
				{
					// if (Robot.isAutoMode())
					// 	if (Robot.lift.getPotPos() > 70 && (speed > 0))
					// 	{
					// 		Robot.intakePivotPiston.retracPivotPiston();
					// 	}
					// if (!Robot.isAutoMode())
					// 	if (Robot.lift.getPotPos() > 15 && (speed > 0))
					// 	{
					// 		Robot.intakePivotPiston.retracPivotPiston();
					// 	}
					// disableBrake();
					driveLiftMotor1(speed);
					driveLiftMotor2(speed);
					// if(Robot.lift.getPotPos() > 0 && Robot.lift.getPotPos() < 30.0)
					// Robot.i2c.write(8, 8);
					// if(Robot.lift.getPotPos() > 30.0 && Robot.lift.getPotPos() < 60.0)
					// Robot.i2c.write(8, 3);
					// if(Robot.lift.getPotPos() > 60.0 && Robot.lift.getPotPos() < 82.5)
					// Robot.i2c.write(8, 11);

				}
				else
				{
					// enableBrake();
					driveLiftMotor1(0.0);
					driveLiftMotor2(0.0);
				}
			}
		}

		isLiftMotor1Failure();
		isLiftMotor2Failure();

		isLiftMotor1BreakerTrip();
		isLiftMotor2BreakerTrip();

		System.out.println("This running?");

	}

	

	/**
	 * The purpose of this method is to compare the current of this motor to that of
	 * the other motors in the same gearbox, if it is less than some percentage of
	 * the others, it is not driving the same and we throw a fault to be checked
	 * later;
	 * 
	 * Once the fault is thrown, it is not reset until the bot is reset.
	 * 
	 * TODO: Write to a file for between bot shutdown persistance;
	 * 
	 * @return
	 */
	private void isLiftMotor1Failure()
	{
		// create a comparison
		double conditionLimtPercent = 0.5;
		if (!this.liftMotor1Fault && this.liftMotor1Voltage >= RobotMap.LIFT_MIN_SPEED)
		{
			this.liftMotor1Fault = ((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) <= conditionLimtPercent
					* Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP)
					&& Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > 2));
		}
	}

	/**
	 * The purpose of this method is to compare the current of this motor to that of
	 * the other motors in the same gearbox, if it is less than some percentage of
	 * the others, it is not driving the same and we throw a fault to be checked
	 * later;
	 * 
	 * Once the fault is thrown, it is not reset until the bot is reset.
	 * 
	 * TODO: Write to a file for between bot shutdown persistance;
	 * 
	 * @return
	 */
	private void isLiftMotor2Failure()
	{
		// create a comparison
		double conditionLimtPercent = 0.5;
		if (!this.liftMotor2Fault && this.liftMotor2Voltage >= RobotMap.LIFT_MIN_SPEED)
		{
			this.liftMotor2Fault = ((Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) <= conditionLimtPercent
					* Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP)
					&& Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) > 2));
		}
	}

	/**
	 * The purpose of this method is to compare the try and determine if we had a
	 * tripped breaker which for our purposes has a signature that while driving the
	 * motor, we see current, then zero current, then sometime later current.
	 * 
	 * If we never see current again, we don't assume it is a tripped breaker but
	 * rather a blown motor captured by the other motor fault
	 * 
	 * This is a special case of the motor fault.
	 * 
	 * TODO: Write to a file for between bot shutdown persistance;
	 * 
	 * @return
	 */
	private void isLiftMotor1BreakerTrip()
	{
		// we are trying to drive motor
		if (this.liftMotor1Voltage >= RobotMap.LIFT_MIN_SPEED)
		{
			// did motor ever get to a high current?
			if (Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) > 15)
				liftMotor1HighCurrent = true;

			if (liftMotor1HighCurrent && Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) < 1)
				liftMotor1HighThenZeroCurrent = true;

			if (!this.isLiftMotor1BreakerTrip && liftMotor1HighThenZeroCurrent)
				this.isLiftMotor1BreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_1_PDP) > 3;

		}
	}

	/**
	 * The purpose of this method is to compare the try and determine if we had a
	 * tripped breaker which for our purposes has a signature that while driving the
	 * motor, we see current, then zero current, then sometime later current.
	 * 
	 * If we never see current again, we don't assume it is a tripped breaker but
	 * rather a blown motor captured by the other motor fault
	 * 
	 * This is a special case of the motor fault.
	 * 
	 * TODO: Write to a file for between bot shutdown persistance;
	 * 
	 * @return
	 */
	private void isLiftMotor2BreakerTrip()
	{
		// we are trying to drive motor
		if (this.liftMotor2Voltage >= RobotMap.LIFT_MIN_SPEED)
		{
			// did motor ever get to a high current?
			if (Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > 35)
				liftMotor2HighCurrent = true;

			if (liftMotor2HighCurrent && Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) < 1)
				liftMotor2HighThenZeroCurrent = true;

			if (!this.isLiftMotor2BreakerTrip && liftMotor2HighThenZeroCurrent)
				this.isLiftMotor2BreakerTrip = Robot.pdp.getChannelCurrent(RobotMap.LIFT_MOTOR_2_PDP) > 3;

		}
	}

	public boolean isSensorValid()
	{
		return this.isSensorValid;
	}

	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveLiftWithJoysticks());
	}
}
