
package org.team2168;

import org.team2168.commands.cargoIntake.IntakeUntilCargo;
import org.team2168.commands.cargoIntake.OperationKeepCargo;
import org.team2168.commands.drivetrain.DisengageDrivetrain;
import org.team2168.commands.drivetrain.DisengageStingers;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.drivetrain.EngageStingers;
import org.team2168.commands.drivetrain.PIDCommands.DriveStingerPIDPath;
import org.team2168.commands.drivetrain.PIDCommands.DriveStingerPIDPath2;
import org.team2168.commands.drivetrain.PIDCommands.EnableLimelight;
import org.team2168.commands.drivetrain.PIDCommands.PauseLimelight;
import org.team2168.commands.hatchProbePistons.DisengageHatchPanel;
import org.team2168.commands.hatchProbePistons.EngageHatchPanel;
import org.team2168.commands.hatchProbePistons.ExtendHatchPlunger;
import org.team2168.commands.hatchProbePistons.IntakeHatchPanel;
import org.team2168.commands.hatchProbePistons.RetractHatchPlunger;
import org.team2168.commands.lift.MoveLiftToCargoShipPosition;
import org.team2168.commands.lift.MoveLiftToLvl1Position;
import org.team2168.commands.lift.MoveLiftToLvl2Position;
import org.team2168.commands.lift.MoveLiftToLvl3Position;
import org.team2168.commands.lift.PIDCommands.DriveLiftPathPIDZZZ;
import org.team2168.commands.monkeyBarPivot.DriveMonkeyBarPivotWithConstant;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPath;
import org.team2168.commands.monkeyBarPivot.PIDCommands.DriveMonkeyBarPivotPIDPathAutoClimb;
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
	public F310 pidTestJoystick = new F310(RobotMap.PID_TEST_JOYSTICK);



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



		/*************************************************************************
		 * Driver Joystick *
		 *************************************************************************/
		driverJoystick.ButtonStart().whenPressed(new EngageStingers()); // add drivetrainshifter
		driverJoystick.ButtonStart().whenPressed(new DisengageDrivetrain());
		driverJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(63));

		driverJoystick.ButtonA().whenPressed(new EngageDrivetrain());
		driverJoystick.ButtonA().whenPressed(new DisengageStingers());

		// driverJoystick.ButtonX().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 5));
		// driverJoystick.ButtonX().whenPressed(new DriveStingerPIDPath(0,24,5));

		driverJoystick.ButtonB().whenPressed(new EngageStingers()); // add drivetrainshifter
		driverJoystick.ButtonB().whenPressed(new DisengageDrivetrain());
		driverJoystick.ButtonB().whenPressed(new DriveMonkeyBarPivotPIDPath(28));

		driverJoystick.ButtonX().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 3.5));
		driverJoystick.ButtonX().whenPressed(new DriveStingerPIDPath(0,25,3));

		driverJoystick.ButtonY().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(28, 0, 2));
		driverJoystick.ButtonY().whenPressed(new DriveStingerPIDPath2(0,10,1.5));
		
		
		driverJoystick.ButtonBack().whenPressed(new DisengageDrivetrain());
		driverJoystick.ButtonBack().whenPressed(new DisengageStingers());

		// driverJoystick.ButtonB().whenPressed(new EnableLimelight());
		// driverJoystick.ButtonB().whenReleased(new PauseLimelight());


		driverJoystick.ButtonLeftStick().whenPressed(new EnableLimelight());
		driverJoystick.ButtonLeftStick().whenReleased(new PauseLimelight());

		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		// if(RobotMap.ENABLE_BUTTON_BOX)
		// {
		/***********************************************************************
		 * Button Box 1
		 ***********************************************************************/
		// buttonBox1.Button1().whenPressed(new ExtendHatchPlunger());
		// buttonBox1.Button2().whenPressed(new EngageHatchPanel());
		// buttonBox1.Button3().whileHeld(new IntakeHatchPanel()); // IR sensor must be tuned
		// buttonBox1.Button3().whenReleased(new RetractHatchPlunger()); 
		// buttonBox1.Button4().whenPressed(new HatchFloorIntakePivotExtend()); //not legit
		// buttonBox1.Button5().whenPressed(new MoveLiftToLvl3Position()); //should move pivot too
		// buttonBox1.Button6().whenPressed(new MoveLiftToLvl2Position()); //should move pivot too
		// buttonBox1.Button7().whenPressed(new MoveLiftToCargoShipPosition()); //not legit
		// buttonBox1.Button8().whenPressed(new MoveLiftToCargoShipPosition()); //not legit
		// buttonBox1.Button9().whenPressed(new MoveLiftToLvl1Position()); //should move pivot too
		// buttonBox1.Button10().whenPressed(new MoveHatchProbePivotTo180Position()); //not legit
		// buttonBox1.Button11().whenPressed(new MoveHatchProbePivotTo0Position()); //not legit
		// buttonBox1.Button12().whenPressed(new MoveHatchProbePivotTo0Position()); //may be legit


		//  /***********************************************************************
		//  * Button Box 2
		//  ***********************************************************************/
		// //buttonBox2.Button1().whenPressed(new ExtendCargoPunch()); apparently no punch
		// // buttonBox2.Button2().whenPressed();
		// // buttonBox2.Button3().whenPressed();
		// buttonBox2.Button4().whenPressed(new DisengageHatchPanel());
		// //buttonBox2.Button5().whenPressed(new defense); //not exist yet
		// //buttonBox2.Button6().whenPressed(new score)); //not exist
		// buttonBox2.Button7().whenPressed(new DriveCargoIntakeWithConstant(-0.5)); //should also spin mb if lift down
		// buttonBox2.Button8().whenPressed(new DriveCargoIntakeWithConstant(-1.0)); //should also spin mb if lift down
		// buttonBox2.Button9().whenPressed(new DriveCargoIntakeWithConstant(1.0)); //should also spin and pivot mb
		// buttonBox2.Button10().whenPressed(new ReleaseHatchPanel()); //not legit
		// //buttonBox2.Button11().whenPressed();
		// buttonBox2.Button12().whenPressed(new RetractHatchPlunger()); 
		// }
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

		operatorJoystick.ButtonRightTrigger().whenReleased(new OperationKeepCargo());

		operatorJoystick.ButtonRightBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(0.7));
		operatorJoystick.ButtonRightBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));
		operatorJoystick.ButtonLeftBumper().whenPressed(new DriveMonkeyBarPivotWithConstant(-0.7));
		operatorJoystick.ButtonLeftBumper().whenReleased(new DriveMonkeyBarPivotWithConstant(0.0));

		//Button X
		operatorJoystick.ButtonX().whenPressed(new ExtendHatchPlunger());
		operatorJoystick.ButtonX().whileHeld(new IntakeHatchPanel());

		//Button A
		operatorJoystick.ButtonA().whenPressed(new RetractHatchPlunger());
		
		//Button Y
		operatorJoystick.ButtonY().whenPressed(new EngageHatchPanel());
		
		//Button B
		operatorJoystick.ButtonB().whenPressed(new DisengageHatchPanel());

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
		// // pidTestJoystick.ButtonY().whenPressed(new PauseLiftPID());
		// // pidTestJoystick.ButtonY().whenPressed(new PauseMonkeyBarPivotPID());
		// // pidTestJoystick.ButtonDownDPad().whenPressed(new MoveLiftToLvl1Position());
		// // pidTestJoystick.ButtonRightDPad().whenPressed(new MoveLiftToLvl2Position());
		// // pidTestJoystick.ButtonUpDPad().whenPressed(new MoveLiftToLvl3Position());
		 pidTestJoystick.ButtonStart().whenPressed(new DriveMonkeyBarPivotPIDPath(40));
		 pidTestJoystick.ButtonBack().whenPressed(new DriveMonkeyBarPivotPIDPath(110));
		 //pidTestJoystick.ButtonY().whenPressed(new DriveMonkeyBarPivotPIDPath(63));
		 //.ButtonA().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 5));
		 //pidTestJoystick.ButtonA().whenPressed(new DriveStingerPIDPath(0,24,5));

		pidTestJoystick.ButtonA().whenPressed(new IntakeUntilCargo());
		pidTestJoystick.ButtonA().whenPressed(new OperationKeepCargo());

		 //pidTestJoystick.ButtonX().whenPressed(new DriveMonkeyBarPivotPIDPathAutoClimb(63, 0, 5));
		 
		//pidTestJoystick.ButtonX().whenPressed(new AutoClimb());

		 pidTestJoystick.ButtonUpDPad().whenPressed(new DriveLiftPathPIDZZZ(35));
		/***********************************************************************
		 * Commands Test Joystick
		 ***********************************************************************/
		// //leds testing
		// pidTestJoystick.ButtonA().whenPressed(new DisabledPattern());
		// pidTestJoystick.ButtonB().whenPressed(new TeleopWithoutGamePiecePattern());
		// pidTestJoystick.ButtonX().whenPressed(new AutoWithoutGamePiecePattern());
		// pidTestJoystick.ButtonY().whenPressed(new HABClimbPattern());
		// pidTestJoystick.ButtonLeftBumper().whenPressed(new PivotingPattern());
		// pidTestJoystick.ButtonRightBumper().whenPressed(new LiftRaisingPattern());
		// pidTestJoystick.ButtonRightTrigger().whenPressed(new LiftLoweringPattern());
		// pidTestJoystick.ButtonDownDPad().whenPressed(new WheelsInPattern());
		// pidTestJoystick.ButtonLeftDPad().whenPressed(new WheelsOutPattern());
		// pidTestJoystick.ButtonRightDPad().whenPressed(new MonkeyBarPattern());
		// pidTestJoystick.ButtonUpDPad().whenPressed(new WithGamePiecePattern());

		
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

			return operatorJoystick.getLeftStickRaw_Y();
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

		return operatorJoystick.getLeftTriggerAxisRaw() - operatorJoystick.getRightTriggerAxisRaw();
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
	
		return -operatorJoystick.getLeftTriggerAxisRaw() + operatorJoystick.getRightTriggerAxisRaw();
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
	 * /
	 * @author Krystina
	 */
	public double getDriveTrainRightJoystick()
	{
		return driverJoystick.getRightStickRaw_Y();
	}

}