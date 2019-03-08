package org.team2168;

import org.team2168.commands.cargoIntake.DriveCargoIntakeWithConstant;
import org.team2168.commands.drivetrain.DisengageDrivetrain;
import org.team2168.commands.drivetrain.DisengageStingers;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.drivetrain.EngageStingers;
import org.team2168.commands.hatchFloorIntake.HatchFloorIntakePivotExtend;
import org.team2168.commands.hatchProbePistons.DisengageHatchPanel;
import org.team2168.commands.hatchProbePistons.EngageHatchPanel;
import org.team2168.commands.hatchProbePistons.ExtendHatchPlunger;
import org.team2168.commands.hatchProbePistons.IntakeHatchPanel;
import org.team2168.commands.hatchProbePistons.ReleaseHatchPanel;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;
import org.team2168.commands.hatchProbePivot.MoveHatchProbePivotTo0Position;
import org.team2168.commands.hatchProbePivot.MoveHatchProbePivotTo180Position;
import org.team2168.commands.hatchProbePivot.PIDCommands.EnableHatchProbePivotPID;
import org.team2168.commands.hatchProbePivot.PIDCommands.PauseHatchProbePivotPID;
import org.team2168.commands.lift.MoveLiftToCargoShipPosition;
import org.team2168.commands.lift.MoveLiftToLvl1Position;
import org.team2168.commands.lift.MoveLiftToLvl2Position;
import org.team2168.commands.lift.MoveLiftToLvl3Position;
import org.team2168.commands.lift.PIDCommands.EnableLiftPIDZZZ;
import org.team2168.commands.lift.PIDCommands.PauseLiftPID;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithConstant;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithConstant;
import org.team2168.commands.monkeyBarPivot.PIDCommands.EnableMonkeyBarPivotPID;
import org.team2168.commands.monkeyBarPivot.PIDCommands.PauseMonkeyBarPivotPID;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;
import org.team2168.utils.TILaunchPad;

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



	public TILaunchPad buttonBox1;
	public TILaunchPad buttonBox2;


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

		buttonBox1 = new TILaunchPad(RobotMap.BUTTON_BOX_1);
		buttonBox2 = new TILaunchPad(RobotMap.BUTTON_BOX_2);
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

		/***********************************************************************
		 * Button Box 1
		 ***********************************************************************/
		buttonBox1.Button1().whenPressed(new ExtendHatchPlunger());
		buttonBox1.Button2().whenPressed(new EngageHatchPanel());
		buttonBox1.Button3().whileHeld(new IntakeHatchPanel()); // IR sensor must be tuned
		buttonBox1.Button3().whenReleased(new RetractHatchPlunger()); 
		buttonBox1.Button4().whenPressed(new HatchFloorIntakePivotExtend()); //not legit
		buttonBox1.Button5().whenPressed(new MoveLiftToLvl3Position()); //should move pivot too
		buttonBox1.Button6().whenPressed(new MoveLiftToLvl2Position()); //should move pivot too
		buttonBox1.Button7().whenPressed(new MoveLiftToCargoShipPosition()); //not legit
		buttonBox1.Button8().whenPressed(new MoveLiftToCargoShipPosition()); //not legit
		buttonBox1.Button9().whenPressed(new MoveLiftToLvl1Position()); //should move pivot too
		buttonBox1.Button10().whenPressed(new MoveHatchProbePivotTo180Position()); //not legit
		buttonBox1.Button11().whenPressed(new MoveHatchProbePivotTo0Position()); //not legit
		buttonBox1.Button12().whenPressed(new MoveHatchProbePivotTo0Position()); //may be legit


		 /***********************************************************************
		 * Button Box 2
		 ***********************************************************************/
		//buttonBox2.Button1().whenPressed(new ExtendCargoPunch()); apparently no punch
		// buttonBox2.Button2().whenPressed();
		// buttonBox2.Button3().whenPressed();
		buttonBox2.Button4().whenPressed(new DisengageHatchPanel());
		//buttonBox2.Button5().whenPressed(new defense); //not exist yet
		//buttonBox2.Button6().whenPressed(new score)); //not exist
		buttonBox2.Button7().whenPressed(new DriveCargoIntakeWithConstant(-0.5)); //should also spin mb if lift down
		buttonBox2.Button8().whenPressed(new DriveCargoIntakeWithConstant(-1.0)); //should also spin mb if lift down
		buttonBox2.Button9().whenPressed(new DriveCargoIntakeWithConstant(1.0)); //should also spin and pivot mb
		buttonBox2.Button10().whenPressed(new ReleaseHatchPanel()); //not legit
		//buttonBox2.Button11().whenPressed();
		buttonBox2.Button12().whenPressed(new RetractHatchPlunger()); 

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
		pidTestJoystick.ButtonA().whenPressed(new EnableLiftPIDZZZ());
		pidTestJoystick.ButtonB().whenPressed(new EnableMonkeyBarPivotPID());
		pidTestJoystick.ButtonX().whenPressed(new EnableHatchProbePivotPID());
		pidTestJoystick.ButtonY().whenPressed(new PauseLiftPID());
		pidTestJoystick.ButtonY().whenPressed(new PauseHatchProbePivotPID());
		pidTestJoystick.ButtonY().whenPressed(new PauseMonkeyBarPivotPID());


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
		return operatorJoystick.getLeftStickRaw_Y() + pidTestJoystick.getLeftStickRaw_Y(); //+ buttonBox1.getAnalogRaw_Channel2();
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
		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();//operatorJoystick.getRightStickRaw_X();
	}

	/*************************************************************************
	 *Monkey Bar Pivot *
	 *************************************************************************/
	public double getMonkeyBarPivotJoystickValue()
	{
		return 0;
	}

	public double getMonkeyBarIntakeJoystickValue()
	{
	
		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();//operatorJoystick.getRightStickRaw_Y();
	}

	/*************************************************************************
	 *Hatch Floor Motor*
	*************************************************************************/
	public double getHatchFloorIntakeJoystickValue()
	{
		return 0;//operatorJoystick.getLeftStickRaw_X();
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

}