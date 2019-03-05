/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.PID.trajectory.OneDimensionalRotation;
import org.team2168.commands.LEDs.AutoWithoutGamePiece;
import org.team2168.commands.LEDs.DisabledPattern;
import org.team2168.commands.LEDs.DisconnectPattern;
import org.team2168.commands.LEDs.HABClimbPattern;
import org.team2168.commands.LEDs.LiftLoweringPattern;
import org.team2168.commands.LEDs.LiftRaisingPattern;
import org.team2168.commands.LEDs.MonkeyBarPattern;
import org.team2168.commands.LEDs.PivotingPattern;
import org.team2168.commands.LEDs.TeleopWithoutGamePiece;
import org.team2168.commands.LEDs.WheelsInPattern;
import org.team2168.commands.LEDs.WheelsOutPattern;
import org.team2168.commands.LEDs.WithGamePiece;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.drivetrain.EngageStingers;
import org.team2168.commands.Drivetrain.EngageDrivetrain;
import org.team2168.commands.Drivetrain.EngageStingers;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
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

	public static F310 driverJoystick = new F310(RobotMap.DRIVER_JOYSTICK);
	public static F310 operatorJoystick = new F310(RobotMap.OPERATOR_JOYSTICK);

	public static F310 driverOperatorEJoystick = new F310(RobotMap.DRIVER_OPERATOR_E_BACKUP);

	public static F310 testJoystick = new F310(RobotMap.COMMANDS_TEST_JOYSTICK);
	public static F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);
	private static LinearInterpolator gunStyleInterpolator;
	private double[][] gunStyleArray = {{-1.0, -1.0},
	                                    {-.15,0.0},
	                                    {.15,0.0},
	                                    {1.0,1.0}};

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() 
  {

		/*************************************************************************
		 *                         Driver Joystick			                       *
		 *************************************************************************/
		driverJoystick.ButtonStart().whenPressed(new EngageDrivetrain());  //???? Which command should
		driverJoystick.ButtonA().whenPressed(new EngageStingers());     //attached to which button
		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);


		/*************************************************************************
		 *                         Operator Joystick         		              *
		 *************************************************************************/



		//leds testing
		testJoystick.ButtonA().whenPressed(new DisabledPattern());
		testJoystick.ButtonB().whenPressed(new TeleopWithoutGamePiece());
		testJoystick.ButtonX().whenPressed(new AutoWithoutGamePiece());
		testJoystick.ButtonY().whenPressed(new HABClimbPattern());
		testJoystick.ButtonLeftTrigger().whenPressed(new DisconnectPattern());
		testJoystick.ButtonLeftBumper().whenPressed(new PivotingPattern());
		testJoystick.ButtonRightBumper().whenPressed(new LiftRaisingPattern());
		testJoystick.ButtonRightTrigger().whenPressed(new LiftLoweringPattern());
		testJoystick.ButtonDownDPad().whenPressed(new WheelsInPattern());
		testJoystick.ButtonLeftDPad().whenPressed(new WheelsOutPattern());
		testJoystick.ButtonRightDPad().whenPressed(new MonkeyBarPattern());
		testJoystick.ButtonUpDPad().whenPressed(new WithGamePiece());
	}

	/**
	 * Returns an instance of the Operator Interface.
	 * 
	 * @return is the current OI object
	 */
	public static OI getInstance() {
		if (instance == null)
			instance = new OI();

		return instance;
	}

	/**
	 * Method that sets that Left side of the drive train so that it drives with
	 * LeftStick Y
	 * 
	 * @author Krystina
	 */
	public static double getDriveTrainLeftJoystick() {
		return driverJoystick.getLeftStickRaw_Y();
	}

	/**
	 * Method that sets that Right side of the drive train so that it drives with
	 * RightStick Y
	 * 
	 * @author Krystina
	 */
	public static double getDriveTrainRightJoystick() {
		return driverJoystick.getRightStickRaw_Y();
	}

	public static double getDriveLiftJoystickValue() {
		return operatorJoystick.getLeftStickRaw_Y() + pidTestJoystick.getLeftStickRaw_Y();
	}

	public static double getFloorIntakeMechanismJoystickValue() {
		return operatorJoystick.getLeftStickRaw_Y();
	}
	public static double getDriveIntakeWheelsJoystickValue() {
		return operatorJoystick.getRightStickRaw_Y();
	}

	public static double getDriveCargoIntakeJoystickValue(){
		return operatorJoystick.getLeftStickRaw_Y();
	}

	public static double getDriveIntakePivotJoystickValue() {
		return testJoystick.getRightStickRaw_Y();
	}

	public static double getGunStyleYValue() {
		// return
		// gunStyleInterpolator.interpolate(Robot.oi.driverJoystick.getLeftStickRaw_X());
		return driverJoystick.getLeftStickRaw_Y();
	}

	public static double getGunStyleXValue() {
		// return
		// gunStyleInterpolator.interpolate(Robot.oi.driverJoystick.getLeftStickRaw_X());
		return -gunStyleInterpolator.interpolate(driverJoystick.getLeftStickRaw_X());
	}
	public static double getDriveWinchJoystickValue() {
		// return
		// gunStyleInterpolator.interpolate(Robot.oi.driverJoystick.getLeftStickRaw_X());
		return operatorJoystick.getRightStickRaw_X();
	}

	public static double getDrivePlungerArmPivotJoystickValue()
	{
		//TODO actually figure out which stick and axis will connect
		return operatorJoystick.getRightStickRaw_Y();
	}
}
