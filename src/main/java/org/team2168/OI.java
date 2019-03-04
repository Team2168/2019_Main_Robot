package org.team2168;

import org.team2168.commands.cargoIntake.DriveCargoIntakeWithJoystick;
import org.team2168.commands.drivetrain.DisengageDrivetrain;
import org.team2168.commands.drivetrain.DisengageStingers;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.drivetrain.EngageStingers;
import org.team2168.commands.hatchProbePistons.DisengageHatchPanel;
import org.team2168.commands.hatchProbePistons.EngageHatchPanel;
import org.team2168.commands.hatchProbePistons.ExtendHatchPlunger;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithConstant;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithJoystick;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithConstant;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI
{
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	private static OI instance = null;

	public F310 driverJoystick = new F310(RobotMap.DRIVER_JOYSTICK);
	public F310 operatorJoystick = new F310(RobotMap.OPERATOR_JOYSTICK);

	// public F310 driverOperatorEJoystick = new
	// F310(RobotMap.DRIVER_OPERATOR_E_BACKUP);

	// public F310 testJoystick = new F310(RobotMap.COMMANDS_TEST_JOYSTICK);
	public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);
	private LinearInterpolator gunStyleInterpolator;
	private double[][] gunStyleArray = { { -1.0, -1.0
			}, { -.15, 0.0
			}, { .15, 0.0
			}, { 1.0, 1.0
			}
	};

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI()
	{

		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		driverJoystick.ButtonStart().whenPressed(new EngageStingers()); // add drivetrainshifter
		driverJoystick.ButtonStart().whenPressed(new DisengageDrivetrain());

		driverJoystick.ButtonA().whenPressed(new EngageDrivetrain());
		driverJoystick.ButtonA().whenPressed(new DisengageStingers());
		
		driverJoystick.ButtonBack().whenPressed(new DisengageDrivetrain());
		driverJoystick.ButtonBack().whenPressed(new DisengageStingers());


		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);


		/*************************************************************************
		 * Operator Joystick *
		 *************************************************************************/



		//////////////// Lower Platform///////////////////////////////////////
		// operatorJoystick.ButtonBack().whenPressed(new LowerPlatform());

		// operatorJoystick.ButtonRightBumper().whileHeld(new DriveCargoIntakeWithConstant(1.0));
		// operatorJoystick.ButtonRightBumper().whileHeld(new DriveMonkeyBarWheelsWithConstant(1.0));
		// operatorJoystick.ButtonLeftBumper().whileHeld(new DriveCargoIntakeWithConstant(-1.0));
		// operatorJoystick.ButtonLeftBumper().whileHeld(new DriveMonkeyBarWheelsWithConstant(-1.0));
		// operatorJoystick.ButtonRightBumper().whenReleased(new DriveCargoIntakeWithConstant(0.0));
		// operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarWheelsWithConstant(0.0));
		// operatorJoystick.ButtonLeftBumper().whenReleased(new DriveCargoIntakeWithConstant(0.0));
		// operatorJoystick.ButtonLeftBumper().whenReleased(new DriveMonkeyBarWheelsWithConstant(0.0));

		operatorJoystick.ButtonRightTrigger().whenPressed(new DriveMonkeyBarIntakeWithJoystick());
		operatorJoystick.ButtonRightTrigger().whenPressed(new DriveCargoIntakeWithJoystick());
		operatorJoystick.ButtonLeftTrigger().whenPressed(new DriveCargoIntakeWithJoystick());
		operatorJoystick.ButtonLeftTrigger().whenPressed(new DriveMonkeyBarIntakeWithJoystick());

		// operatorJoystick.ButtonRightTrigger().whenPressed(new DriveRotateMonkeyBarWithJoystick());
		// operatorJoystick.ButtonLeftTrigger().whenPressed(new DriveRotateMonkeyBarWithJoystick());

		operatorJoystick.ButtonRightBumper().whileHeld(new DriveMonkeyBarPivotWithConstant(0.75));
		operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));
		operatorJoystick.ButtonLeftBumper().whileHeld(new DriveMonkeyBarPivotWithConstant(-0.75));
		operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));

		operatorJoystick.ButtonStart().whileHeld(new DriveMonkeyBarIntakeWithConstant(0.2));
		operatorJoystick.ButtonStart().whenReleased(new DriveMonkeyBarIntakeWithConstant(0.0));
		operatorJoystick.ButtonBack().whileHeld(new DriveMonkeyBarIntakeWithConstant(-0.2));
		operatorJoystick.ButtonBack().whenReleased(new DriveMonkeyBarIntakeWithConstant(0.0));

		operatorJoystick.ButtonY().whenPressed(new ExtendHatchPlunger());
		operatorJoystick.ButtonB().whenPressed(new RetractHatchPlunger());
		operatorJoystick.ButtonA().whenPressed(new EngageHatchPanel());
		operatorJoystick.ButtonX().whenPressed(new DisengageHatchPanel());

		/////////////// Intake and pivot up
		/////////////// afterwards/////////////////////////////////////////////////////////////////////////



		/// End game actuations//////////////////////////////////////////


		// operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());

		//////////////// Prepare to climb/////////////////////////////
		// operatorJoystick.ButtonBack().whenPressed(new EnableRachet());
		// operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());

		//////////////// Lift Pid
		//////////////// commands////////////////////////////////////////////////////
		// pidTestJoystick.ButtonA().whenPressed(new Drive14FeetForward_9FeetLeft());
		// pidTestJoystick.ButtonB().whenPressed(new Drive10FeetBackward());


	}

	/**
	 * Returns an instance of the Operator Interface.
	 * 
	 * @return is the current OI object
	 */
	public static OI getInstance()
	{
		if (instance == null)
			instance = new OI();

		return instance;
	}

	/*************************************************************************
	 * Lift *
	 *************************************************************************/

	public double getLiftJoystickValue()
	{
		return operatorJoystick.getLeftStickRaw_Y() + pidTestJoystick.getLeftStickRaw_Y();
	}

	/*************************************************************************
	 * Hatch Probe Pivot *
	 *************************************************************************/
	public double getHatchProbePivotJoystickValue()
	{
		return operatorJoystick.getRightStickRaw_Y();
	}

	public double getCargoIntakeJoystickValue()
	{
		return 0;//operatorJoystick.getRightStickRaw_X();
	}

	/*************************************************************************
	 *Monkey Bar Pivot *
	 *************************************************************************/
	public double getMonkeyBarPivotJoystickValue()
	{
		return 0;
	}

	public double getHatchFloorIntakeJoystickValue()
	{
		return 0;//operatorJoystick.getLeftStickRaw_X();
	}

	public double getDriveIntakeWheelsJoystickValue()
	{
	
		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();//operatorJoystick.getRightStickRaw_Y();
	}

	public double getDriveIntakePivotJoystickValue()
	{
		return 0;
	}
	
	/*************************************************************************
	 * Drivetrain *
	 *************************************************************************/

	public double getGunStyleXValue()
	{
		return driverJoystick.getLeftStickRaw_X();
	}

	public double getGunStyleYValue()
	{

		return gunStyleInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
	}

	public double getDriveWinchJoystickValue()
	{
		return operatorJoystick.getRightStickRaw_X();
	}

	/**
	 * Method that sets that Left side of the drive train so that it drives with
	 * LeftStick Y
	 * 
	 * @author Krystina
	 */
	public double getDriveTrainLeftJoystick()
	{
		return driverJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Method that sets that Right side of the drive train so that it drives with
	 * RightStick Y
	 * 
	 * @author Krystina
	 */
	public double getDriveTrainRightJoystick()
	{
		return driverJoystick.getRightStickRaw_Y();
	}

	public double getDriveMonkeyBarPivotJoystickValue() {
		return operatorJoystick.getLeftStickRaw_X();
	}

}
