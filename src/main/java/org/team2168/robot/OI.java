package org.team2168.robot;


import org.team2168.commands.drivetrain.DriveWithJoystick;
import org.team2168.commands.drivetrain.ShiftHigh;
import org.team2168.commands.drivetrain.ShiftLow;


import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;
import org.team2168.utils.consoleprinter.ConsolePrinter;


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

	private static LinearInterpolator gunStyleInterpolator;
	private double[][] gunStyleArray = {{-1.0, -1.0},
	                                    {-.15,0.0},
	                                    {.15,0.0},
	                                    {1.0,1.0}};
			
	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() {

		/*************************************************************************
		 *                         Driver Joystick			                       *
		 *************************************************************************/
		driverJoystick.ButtonStart().whenPressed(new ShiftLow());  //add drivetrainshifter
		driverJoystick.ButtonA().whenPressed(new ShiftHigh());
		gunStyleInterpolator = new LinearInterpolator(gunStyleArray);

		////////////// Operator Joystick//////////////
		
		
		/*************************************************************************
		 *                         Operator Joystick         		              *
		  *************************************************************************/

		// pidTestJoystick.ButtonLeftTrigger().whenPressed(new OpenIntake());
		// pidTestJoystick.ButtonLeftBumper().whenPressed(new CloseIntake());
		
		// pidTestJoystick.ButtonUpDPad().whenReleased(new PivotIntakeUp());
		// pidTestJoystick.ButtonDownDPad().whenReleased(new PivotIntakeDown());
		
		
		

		// /////////////////Emergency Lower Intake/////////////////////////////////////////
		// operatorJoystick.ButtonUpDPad().whenReleased(new PivotIntakeUp());
		// operatorJoystick.ButtonDownDPad().whenReleased(new PivotIntakeDown());
		
		// operatorJoystick.ButtonRightDPad().whenPressed(new EngageIntakePivotHardStop());
		// operatorJoystick.ButtonLeftDPad().whenPressed(new DisEngageIntakePivotHardStop());
		
		// ////////////////Intake Cube and lift to exchange////////////////////////////////////////////////////
		// //operatorJoystick.ButtonRightTrigger().whileHeld(new RotatePivotDownAutomatically(-RobotMap.CUBE_PIVOT_DOWN_CONSTANT));
		// operatorJoystick.ButtonRightTrigger().whenPressed(new EngageIntakePivotHardStop());
		// operatorJoystick.ButtonRightTrigger().whenPressed(new IntakeUntilCube());

		//operatorJoystick.ButtonRightTrigger().whenReleased(new DriveIntakeWheelsWithConstant(0.0));
		//operatorJoystick.ButtonLeftTrigger().whenReleased(new DriveLiftPIDZZZ(10.0, 0.5, 0.16,1.0,true));
		
		///////////////Intake and pivot up afterwards/////////////////////////////////////////////////////////////////////////
		
		//operatorJoystick.ButtonLeftTrigger().whileHeld(new IntakeUntilCubeAndPivotUp());
		
//		operatorJoystick.ButtonLeftTrigger().whileHeld(new IntakeUntilCube());
//		operatorJoystick.ButtonLeftTrigger().whileHeld(new RotatePivotDownAutomatically(-RobotMap.CUBE_PIVOT_DOWN_CONSTANT));
//	    operatorJoystick.ButtonLeftTrigger().whenPressed(new CloseIntake()); //open for comp bot
//		operatorJoystick.ButtonLeftTrigger().whenReleased(new CloseIntake());
//		operatorJoystick oooooo.ButtonLeftTrigger().whenReleased(new RotatePivotUpAutomatically(RobotMap.CUBE_PIVOT_CONSTANT_NO_CUBE));
//		operatorJoystick.ButtonLeftTrigger().whenReleased(new DriveIntakeWheelsWithConstant(0.0));
		

		
		
		
		///End game actuations//////////////////////////////////////////
		//operatorJoystick.ButtonStart().whenPressed(new LiftShiftHigh());

		
		//operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());
		
		////////////////Prepare to climb/////////////////////////////
		//operatorJoystick.ButtonBack().whenPressed(new EnableRachet());
		//operatorJoystick.ButtonBack().whenPressed(new LiftShiftLow());
		

		////////////////Lift Pid commands////////////////////////////////////////////////////
//		operatorJoystick.ButtonY().whenPressed(new DriveLiftPIDZZZ(87.0, 0.5, 0.16,1.0,true));
		// operatorJoystick.ButtonY().whenPressed(new FadeAway());
		// operatorJoystick.ButtonA().whenPressed(new LiftShiftLow());
		// operatorJoystick.ButtonX().whenPressed(new DriveLiftPIDZZZ(1.5, 0.7, 0.16,0.5,true));
		// operatorJoystick.ButtonB().whenPressed(new LiftShiftHigh());
		////////////////Raise platform/////////////////////////////
		
		//operatorJoystick.ButtonA().whenPressed(new RaisePlatform());
		
		//////////////// Open arm and shift high and disengage rachet ///
		//operatorJoystick.ButtonB().whenPressed(new OpenGuidingArm());
		//operatorJoystick.ButtonB().whenPressed(new LiftShiftHigh());
		//operatorJoystick.ButtonB().whenPressed(new DisableRachet());
		
		////////////////For testing purposes//////////////////////
		//testJoystick.ButtonRightTrigger().whenPressed(new RobotPrep());
		//testJoystick.ButtonLeftTrigger().whenPressed(new DriveToLeftScale2CubeFromLeftSideV2());
		
		
		//testJoystick.ButtonLeftBumper().whenPressed(new test());	
		//testJoystick.ButtonLeftTrigger().whenPressed(new DisEngageIntakePivotHardStop());
		
		
		//testJoystick.ButtonA().whenPressed(new EnableRachet());
		//testJoystick.ButtonB().whenPressed(new DisableRachet());
		//testJoystick.ButtonLeftDPad().whenPressed(new EnableBrake());
		//testJoystick.ButtonRightDPad().whenPressed(new DisableBrake());

//		testJoystick.ButtonA().whenPressed(new RotateXDistancePIDZZZ(45,0.5,0.2));
//		testJoystick.ButtonB().whenPressed(new RotateXDistancePIDZZZ(-45,0.5,0.2));
//		testJoystick.ButtonX().whenPressed(new RotateXDistancePIDZZZ(90,0.5,0.2));
//		
//		testJoystick.ButtonY().whenPressed(new RotateXDistancePIDZZZ(0,0.5,0.2));
//		testJoystick.ButtonUpDPad().whenPressed(new EnableRotateXDistancePIDZZZ(0));
//		testJoystick.ButtonDownDPad().whenPressed(new DrivePIDPause());
//
		//testJoystick.ButtonRightTrigger().whenPressed(new DrivePIDPathQuintic(Robot.leftVelPathQuintic, Robot.rightVelPathQuintic));
//		testJoystick.ButtonRightBumper().whenPressed(new DriveIntakeWheelsWithConstant(-1));
//		
//		testJoystick.ButtonDownDPad().whenPressed(new RotatePivotUpAutomatically(-RobotMap.CUBE_PIVOT_DOWN_CONSTANT));
//		testJoystick.ButtonUpDPad().whenPressed(new RotatePivotUpAutomatically(RobotMap.CUBE_PIVOT_DOWN_CONSTANT));
//		
//		testJoystick.ButtonLeftBumper().whenPressed(new OpenIntake());
//		testJoystick.ButtonLeftTrigger().whenPressed(new OpenIntake());
//		///////////////PID testing//////////////////////////////////////////////////////

		
		//pidTestJoystick.ButtonB().whenPressed(new RotateXDistancePIDZZZ(-45,0.5,0.2));
		//pidTestJoystick.ButtonX().whenPressed(new DriveToRightScaleFromLeftSide());
		//pidTestJoystick.ButtonY().whenPressed(new DriveToLeftScaleFromLeftSide());
//		pidTestJoystick.ButtonA().whileHeld(new  DrivePIDPathQuintic(0, -45, 2500, 3000, 30000));//rotate A to B
//		pidTestJoystick.ButtonB().whileHeld(new  DrivePIDPathQuintic(90, 90, 2500, 3000, 30000));//rotate A to B
		//pidTestJoystick.ButtonB().whenPressed(new RotateXDistancePIDZZZ(142,0.6,0.2,0.5,true));
		
		
//		pidTestJoystick.ButtonY().whileHeld(new DriveIntakeWheelsWithConstant(-0.5));
		//pidTestJoystick.ButtonY().whenPressed(new DriveLiftPIDZZZ(74.0, 0.9, 0.1,1.0,true));
		
// 		pidTestJoystick.ButtonY().whenPressed(new DriveStraight10Feet());
// 		pidTestJoystick.ButtonA().whenPressed(new LeftExchange_BackLeftRocket());
// 		pidTestJoystick.ButtonX().whenPressed(new FrontLeftRocket_LeftExchange());
// 		pidTestJoystick.ButtonY().whenPressed(new LeftPlatform_FrontLeftRocket());
// 		pidTestJoystick.ButtonB().whenPressed(new DriveStraight10FeetBackwards_quintic());
//		pidTestJoystick.ButtonX().whenPressed(new RightStart_FrontRightCargo());
//		pidTestJoystick.ButtonY().whenPressed(new DriveStraight10Feet_Left2Feet());
		
		//pidTestJoystick.ButtonUpDPad().whenPressed(new DriveLiftPIDZZZ(2.0, 0.5, 0.16,1.0,true));
		
		//Light Testing//////////////////////////////////////////////////////// 
		//testJoystick.ButtonX().whenPressed(new TestAuto2());
//		testJoystick.ButtonA().whenPressed(new DisabledPattern());
//		testJoystick.ButtonB().whenPressed(new TeleopWithoutCube());
//		testJoystick.ButtonX().whenPressed(new AutoWithoutCube());
//		testJoystick.ButtonY().whenPressed(new ClimbingPattern());
//		testJoystick.ButtonLeftTrigger().whenPressed(new DisconnectPattern());
//		testJoystick.ButtonLeftBumper().whenPressed(new ForksDeployedPattern());
//		testJoystick.ButtonRightBumper().whenPressed(new LiftHigh());
//		testJoystick.ButtonRightTrigger().whenPressed(new LiftMed());
//		testJoystick.ButtonDownDPad().whenPressed(new LiftLow());
//		testJoystick.ButtonLeftDPad().whenPressed(new SpitPattern());
//		testJoystick.ButtonRightDPad().whenPressed(new SuckPattern());
//		testJoystick.ButtonUpDPad().whenPressed(new WithCubePattern());
		
		
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

	
}
