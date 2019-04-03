
package org.team2168;

import org.team2168.PID.trajectory.OneDimensionalRotation;
import org.team2168.commands.LEDs.AutoWithoutGamePiecePattern;
import org.team2168.commands.LEDs.DisabledPattern;
import org.team2168.commands.LEDs.HABClimbPattern;
import org.team2168.commands.LEDs.LiftLoweringPattern;
import org.team2168.commands.LEDs.LiftRaisingPattern;
import org.team2168.commands.LEDs.MonkeyBarPattern;
import org.team2168.commands.LEDs.PivotingPattern;
import org.team2168.commands.LEDs.TeleopWithoutGamePiece;
import org.team2168.commands.LEDs.WheelsInPattern;
import org.team2168.commands.LEDs.WheelsOutPattern;
import org.team2168.commands.LEDs.WithGamePiecePattern;
import org.team2168.commands.auto.MoveToIntakePosition;
import org.team2168.commands.auto.RotateAndMoveLiftCargoShip;
import org.team2168.commands.cargoIntake.DriveCargoIntakeWithJoystick;
import org.team2168.commands.cargoIntake.DriveCargoIntakeWithConstant;
import org.team2168.commands.drivetrain.DisengageDrivetrain;
import org.team2168.commands.drivetrain.DisengageStingers;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.drivetrain.EngageStingers;
import org.team2168.commands.drivetrain.PIDCommands.EnableLimelight;
import org.team2168.commands.drivetrain.PIDCommands.PauseLimelight;
import org.team2168.commands.hatchFloorIntake.IntakeFloorHatch;
import org.team2168.commands.hatchProbePistons.DisengageHatchPanel;
import org.team2168.commands.hatchProbePistons.EngageHatchPanel;
import org.team2168.commands.hatchProbePistons.ExtendHatchPlunger;
import org.team2168.commands.hatchProbePistons.IntakeHatchPanel;
import org.team2168.commands.hatchProbePistons.ReadyToIntake;
import org.team2168.commands.hatchProbePistons.ReleaseHatchPanel;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;
import org.team2168.commands.lift.MoveLiftToCargoShipPosition;
import org.team2168.commands.lift.MoveLiftToLvl1Position;
import org.team2168.commands.lift.MoveLiftToLvl2Position;
import org.team2168.commands.lift.MoveLiftToLvl3Position;
import org.team2168.commands.monkeyBarIntakeWheels.DriveMonkeyBarIntakeWithConstant;
import org.team2168.commands.monkeyBarIntakeWheels.SpinMBWhenLiftDown;
import org.team2168.commands.lift.PIDCommands.DriveLiftPathPIDZZZ;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithConstant;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPath;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToCargoIntakePosition;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToFloorPosition;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToSafePositionForPivot;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToSafePositionForScoring;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToStowPosition;
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
	public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);

	public TILaunchPad buttonBox1;
	public TILaunchPad buttonBox2;

	// public F310 driverOperatorEJoystick = new
	// F310(RobotMap.DRIVER_OPERATOR_E_BACKUP);

	// public F310 testJoystick = new F310(RobotMap.COMMANDS_TEST_JOYSTICK);
	//public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);
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

		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBox1 = new TILaunchPad(RobotMap.BUTTON_BOX_1);
			buttonBox2 = new TILaunchPad(RobotMap.BUTTON_BOX_2);
		}
		

		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		driverJoystick.ButtonStart().whenPressed(new EngageStingers()); // add drivetrainshifter
		driverJoystick.ButtonStart().whenPressed(new DisengageDrivetrain());

		driverJoystick.ButtonA().whenPressed(new EngageDrivetrain());
		driverJoystick.ButtonA().whenPressed(new DisengageStingers());
		
		driverJoystick.ButtonBack().whenPressed(new DisengageDrivetrain());
		driverJoystick.ButtonBack().whenPressed(new DisengageStingers());

		driverJoystick.ButtonB().whenPressed(new EnableLimelight());
		driverJoystick.ButtonB().whenReleased(new PauseLimelight());

		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		if(RobotMap.ENABLE_BUTTON_BOX)
		{
		/***********************************************************************
		 * Button Box 1
		 ***********************************************************************/
		buttonBox1.Button1().whenPressed(new ExtendHatchPlunger());
		buttonBox1.Button2().whenPressed(new EngageHatchPanel());
		buttonBox1.Button3().whileHeld(new ReadyToIntake());
		buttonBox1.Button3().whenReleased(new IntakeHatchPanel());
		buttonBox1.Button4().whenPressed(new IntakeFloorHatch()); //not legit
		// buttonBox1.Button5().whenPressed(new RotateAndMoveLiftLevel3()); no pivot PID yet
		// buttonBox1.Button6().whenPressed(new RotateAndMoveLiftLevel2());
		// buttonBox1.Button7().whenPressed(new RotateAndMoveLiftCargoShipFrontPosition());
		// buttonBox1.Button8().whenPressed(new RotateAndMoveLiftCargoShip());
		// buttonBox1.Button9().whenPressed(new RotateAndMoveLiftLevel1());
		buttonBox1.Button5().whenPressed(new MoveLiftToLvl3Position());
		buttonBox1.Button6().whenPressed(new MoveLiftToLvl2Position());
		// buttonBox1.Button6().whenPressed(new RotateAndMoveLiftCargoShipFrontPosition());
		buttonBox1.Button8().whenPressed(new MoveLiftToCargoShipPosition());
		buttonBox1.Button9().whenPressed(new MoveLiftToLvl1Position());
		// buttonBox1.Button10().whenPressed(new MoveHatchProbePivotToOppositeSide()); //not legit
		// buttonBox1.Button11().whenPressed(new MoveHatchProbePivotTo180Position()); //not legit
		// buttonBox1.Button12().whenPressed(new MoveHatchProbePivotTo0Position()); //may be legit


		 /***********************************************************************
		 * Button Box 2
		 ***********************************************************************/
		//buttonBox2.Button1().whenPressed(new ExtendCargoPunch()); apparently no punch
		// buttonBox2.Button2().whenPressed();
		// buttonBox2.Button3().whenPressed();
		buttonBox2.Button4().whenPressed(new DisengageHatchPanel());
		//buttonBox2.Button5().whenPressed(new DefenseInsideFramePerimeter()); //untested
		//buttonBox2.Button6().whenPressed(new score)); //not exist
		buttonBox2.Button7().whenPressed(new DriveCargoIntakeWithConstant(-0.5));
		buttonBox2.Button7().whenReleased(new DriveCargoIntakeWithConstant(0.0)); //implemented same system for stopping as mb pivot below
		buttonBox2.Button7().whenPressed(new SpinMBWhenLiftDown(-0.5));
		buttonBox2.Button7().whenReleased(new SpinMBWhenLiftDown(0.0));

		buttonBox2.Button8().whenPressed(new DriveCargoIntakeWithConstant(-1.0)); 
		buttonBox2.Button8().whenReleased(new DriveCargoIntakeWithConstant(0.0));
		buttonBox2.Button8().whenPressed(new SpinMBWhenLiftDown(-0.5));
		buttonBox2.Button8().whenReleased(new SpinMBWhenLiftDown(0.0));

		buttonBox2.Button9().whenPressed(new DriveCargoIntakeWithConstant(0.5)); 
		buttonBox2.Button9().whenPressed(new DriveCargoIntakeWithConstant(0.0)); 
		buttonBox2.Button9().whenPressed(new DriveMonkeyBarIntakeWithConstant(0.5)); //todo: make sure these are the speeds we want
		buttonBox2.Button9().whenReleased(new DriveMonkeyBarIntakeWithConstant(0.0)); //todo: make sure these are the speeds we want
		//buttonBox2.Button9().whenPressed(new MoveMonkeyBarToCargoIntakePosition()); 
		
		buttonBox2.Button10().whileHeld(new ExtendHatchPlunger());
		buttonBox2.Button10().whenReleased(new ReleaseHatchPanel());
		//buttonBox2.Button11().whenPressed();
		buttonBox2.Button12().whenPressed(new RetractHatchPlunger()); 
		}
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

		
		operatorJoystick.ButtonDownDPad().whenPressed(new MoveLiftToLvl1Position());
		operatorJoystick.ButtonRightDPad().whenPressed(new MoveLiftToLvl2Position());
		operatorJoystick.ButtonUpDPad().whenPressed(new MoveLiftToLvl3Position());
		operatorJoystick.ButtonLeftDPad().whenPressed(new MoveLiftToCargoShipPosition());

		operatorJoystick.ButtonRightBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(0.7));
		operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));
		operatorJoystick.ButtonLeftBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(-0.7));
		operatorJoystick.ButtonLeftBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));


		operatorJoystick.ButtonX().whenPressed(new ExtendHatchPlunger());
		operatorJoystick.ButtonX().whileHeld(new IntakeHatchPanel());
		operatorJoystick.ButtonA().whenPressed(new RetractHatchPlunger());
		operatorJoystick.ButtonB().whenPressed(new EngageHatchPanel());
		operatorJoystick.ButtonY().whenPressed(new DisengageHatchPanel());

		operatorJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(40));
		//operatorJoystick.ButtonStart().whenPressed(new IntakeUntilCargoAndPivot());
		operatorJoystick.ButtonBack().whenPressed(new DriveMonkeyBarPivotPIDPath(110));


		

		/////////////// Intake and pivot up
		/////////////// afterwards/////////////////////////////////////////////////////////////////////////



		/// End game actuations//////////////////////////////////////////


		// operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());

		//////////////// Prepare to climb/////////////////////////////
		// operatorJoystick.ButtonBack().whenPressed(new EnableRachet());
		// operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());

		//////////////// Lift Pid
		//////////////// commands////////////////////////////////////////////////////
		// pidTestJoystick.ButtonDownDPad().whenPressed(new DriveLiftPathPIDZZZ(RobotMap.LIFT_LVL_1_POS));
		// pidTestJoystick.ButtonRightDPad().whenPressed(new DriveLiftPathPIDZZZ(RobotMap.LIFT_LVL_2_POS));
		// pidTestJoystick.ButtonUpDPad().whenPressed(new DriveLiftPathPIDZZZ(RobotMap.LIFT_LVL_3_POS));
		// pidTestJoystick.ButtonLeftDPad().whenPressed(new DriveLiftPathPIDZZZ(RobotMap.LIFT_CARGO_SHIP_POS));
		
		// pidTestJoystick.ButtonY().whenPressed(new PauseLiftPID());
		// pidTestJoystick.ButtonY().whenPressed(new PauseMonkeyBarPivotPID());
		
		//automated hatch intake
		// if(HatchProbePistons.getInstance().isHatchEngaged())
		// {
		// 	pidTestJoystick.ButtonX().whileHeld(new ExtendHatchPlunger());
		// 	pidTestJoystick.ButtonX().whenReleased(new ReleaseHatchPanel());
		// }
		// else
		// {
		// 	pidTestJoystick.ButtonX().whileHeld(new ReadyToIntake());
		// 	pidTestJoystick.ButtonX().whenReleased(new IntakeHatchPanel());
		// }

		// pidTestJoystick.ButtonY().whenPressed(new RotateAndMoveLiftLevel3());
		// pidTestJoystick.ButtonB().whenPressed(new RotateAndMoveLiftLevel2());
		// pidTestJoystick.ButtonA().whenPressed(new RotateAndMoveLiftLevel1());
		
		pidTestJoystick.ButtonX().whenPressed(new MoveMonkeyBarToStowPosition());
		pidTestJoystick.ButtonY().whenPressed(new MoveMonkeyBarToSafePositionForPivot());
		pidTestJoystick.ButtonB().whenPressed(new MoveMonkeyBarToCargoIntakePosition());
		pidTestJoystick.ButtonA().whenPressed(new MoveMonkeyBarToFloorPosition());

		pidTestJoystick.ButtonDownDPad().whenPressed(new MoveToIntakePosition());
		pidTestJoystick.ButtonRightDPad().whenPressed(new RotateAndMoveLiftCargoShip());

		// pidTestJoystick.ButtonDownDPad().whenPressed(new MoveLiftToLvl1Position());
		// pidTestJoystick.ButtonRightDPad().whenPressed(new MoveLiftToLvl2Position());
		// pidTestJoystick.ButtonUpDPad().whenPressed(new MoveLiftToLvl3Position());
		pidTestJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(40));
		// pidTestJoystick.ButtonB().whenPressed(new IntakeUntilCargoAndPivot());
		pidTestJoystick.ButtonBack().whenPressed(new DriveMonkeyBarPivotPIDPath(100));

		//  pidTestJoystick.ButtonA().whileHeld(new DriveMonkeyBarIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveCargoIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveMonkeyBarIntakeWithConstant(0.80));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveCargoIntakeWithConstant(0.8));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveMonkeyBarIntakeWithConstant(0.6));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveCargoIntakeWithConstant(0.6));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveMonkeyBarIntakeWithConstant(0.4));
		//  pidTestJoystick.ButtonA().whileHeld(new DriveCargoIntakeWithConstant(0.4));

		//  pidTestJoystick.ButtonDownDPad().whileHeld(new DriveMonkeyBarIntakeWithConstant(0.5));
		//  pidTestJoystick.ButtonDownDPad().whileHeld(new DriveCargoIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonUpDPad().whileHeld(new DriveMonkeyBarIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonUpDPad().whileHeld(new DriveCargoIntakeWithConstant(0.5));
		//  pidTestJoystick.ButtonRightDPad().whileHeld(new DriveMonkeyBarIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonRightDPad().whileHeld(new DriveCargoIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonLeftDPad().whileHeld(new DriveMonkeyBarIntakeWithConstant(1.0));
		//  pidTestJoystick.ButtonLeftDPad().whileHeld(new DriveCargoIntakeWithConstant(1.0));
		 
		 
		/***********************************************************************
		 * Commands Test Joystick
		 ***********************************************************************/
		// //leds testing
		// testJoystick.ButtonA().whenPressed(new DisabledPattern());
		// testJoystick.ButtonB().whenPressed(new TeleopWithoutGamePiece());
		// testJoystick.ButtonX().whenPressed(new AutoWithoutGamePiecePattern());
		// testJoystick.ButtonY().whenPressed(new HABClimbPattern());
		// testJoystick.ButtonLeftBumper().whenPressed(new PivotingPattern());
		// testJoystick.ButtonRightBumper().whenPressed(new LiftRaisingPattern());
		// testJoystick.ButtonRightTrigger().whenPressed(new LiftLoweringPattern());
		// testJoystick.ButtonDownDPad().whenPressed(new WheelsInPattern());
		// testJoystick.ButtonLeftDPad().whenPressed(new WheelsOutPattern());
		// testJoystick.ButtonRightDPad().whenPressed(new MonkeyBarPattern());
		// testJoystick.ButtonUpDPad().whenPressed(new WithGamePiecePattern());


		// // pidTestJoystick.ButtonY().whenPressed(new PauseLiftPID());
		// // pidTestJoystick.ButtonY().whenPressed(new PauseMonkeyBarPivotPID());

		// // pidTestJoystick.ButtonDownDPad().whenPressed(new MoveLiftToLvl1Position());
		// // pidTestJoystick.ButtonRightDPad().whenPressed(new MoveLiftToLvl2Position());
		// // pidTestJoystick.ButtonUpDPad().whenPressed(new MoveLiftToLvl3Position());
		 pidTestJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(40));
		 pidTestJoystick.ButtonBack().whenPressed(new DriveMonkeyBarPivotPIDPath(110));

		 pidTestJoystick.ButtonUpDPad().whenPressed(new DriveLiftPathPIDZZZ(35));

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
	//correst
	public double getLiftJoystickValue()
	{
		double buttonBoxVal = 0.0;
		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBoxVal = buttonBox1.getAnalogRaw_Channel1();
		}
		return operatorJoystick.getLeftStickRaw_Y() - buttonBoxVal;
	}

	/*************************************************************************
	 * Hatch Probe Pivot *
	 *************************************************************************/
	public double getHatchProbePivotJoystickValue()
	{
		double buttonBoxVal = 0.0;
		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBoxVal = buttonBox1.getAnalogRaw_Channel0();
		}
		return operatorJoystick.getRightStickRaw_Y() + buttonBoxVal;
	}

	public double getCargoIntakeJoystickValue()
	{
		double buttonBoxVal = 0.0;
		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBoxVal = buttonBox2.getAnalogRaw_Channel1();
		}
		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw() - buttonBoxVal;
	}

	/*************************************************************************
	 *Monkey Bar Pivot *
	 *************************************************************************/
	public double getMonkeyBarPivotJoystickValue()
	{
		double buttonBoxVal = 0.0;
		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBoxVal = buttonBox2.getAnalogRaw_Channel0();
		}
		return 0 + buttonBoxVal;
	}

	public double getMonkeyBarIntakeJoystickValue()
	{
		double buttonBoxVal = 0.0;
		if (RobotMap.ENABLE_BUTTON_BOX)
		{
			buttonBoxVal = buttonBox2.getAnalogRaw_Channel1();
		}
		return -operatorJoystick.getLeftTriggerAxisRaw() + operatorJoystick.getRightTriggerAxisRaw() + buttonBoxVal;
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

	/**
	 * @return the driverJoystick
	 */
	public F310 getDriverJoystick()
	{
		return driverJoystick;
	}

	/**
	 * @param driverJoystick the driverJoystick to set
	 */
	public void setDriverJoystick(F310 driverJoystick)
	{
		this.driverJoystick = driverJoystick;
	}

	/**
	 * @return the operatorJoystick
	 */
	public F310 getOperatorJoystick()
	{
		return operatorJoystick;
	}

	/**
	 * @param operatorJoystick the operatorJoystick to set
	 */
	public void setOperatorJoystick(F310 operatorJoystick)
	{
		this.operatorJoystick = operatorJoystick;
	}

	/**
	 * @return the pidTestJoystick
	 */
	public F310 getPidTestJoystick()
	{
		return pidTestJoystick;
	}

	/**
	 * @param pidTestJoystick the pidTestJoystick to set
	 */
	public void setPidTestJoystick(F310 pidTestJoystick)
	{
		this.pidTestJoystick = pidTestJoystick;
	}

	/**
	 * @return the gunStyleInterpolator
	 */
	public LinearInterpolator getGunStyleInterpolator()
	{
		return gunStyleInterpolator;
	}

	/**
	 * @param gunStyleInterpolator the gunStyleInterpolator to set
	 */
	public void setGunStyleInterpolator(LinearInterpolator gunStyleInterpolator)
	{
		this.gunStyleInterpolator = gunStyleInterpolator;
	}

	/**
	 * @return the gunStyleArray
	 */
	public double[][] getGunStyleArray()
	{
		return gunStyleArray;
	}

	/**
	 * @param gunStyleArray the gunStyleArray to set
	 */
	public void setGunStyleArray(double[][] gunStyleArray)
	{
		this.gunStyleArray = gunStyleArray;
	}

}