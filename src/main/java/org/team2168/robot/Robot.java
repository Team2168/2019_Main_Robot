package org.team2168.robot;

import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;
import org.team2168.PID.trajectory.QuinticTrajectory;
import org.team2168.commands.auto.AS.DriveStraight10Feet;
import org.team2168.commands.auto.AS.DriveStraight2Feet;
import org.team2168.commands.auto.AS.DriveStraight4Feet;
import org.team2168.commands.auto.AS.DriveStraight6Feet;
import org.team2168.commands.auto.AS.DriveStraight8Feet;
// import org.team2168.commands.auto.RealOnes.DriveStraight;
// import org.team2168.commands.auto.RealOnes.DriveToLeftScale3CubeFromLeftSide;
// import org.team2168.commands.auto.RealOnes.DriveToLeftScaleAndLeftSwitchV2;
// import org.team2168.commands.auto.RealOnes.DriveToLeftScaleOnlyV2;
// import org.team2168.commands.auto.RealOnes.DriveToLeftSwitchAndRightScaleFromLeft;
// import org.team2168.commands.auto.RealOnes.DriveToRightScaleFromLeft;
// import org.team2168.commands.auto.RealOnes.NeverRunMe;
// import org.team2168.commands.auto.RealOnes.TestAuto;
// import org.team2168.commands.auto.RightSide.DriveToScale2CubeFromRightSide;
// import org.team2168.commands.auto.selector.AutoStartCenter2Cube;
import org.team2168.commands.pneumatics.StartCompressor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Pneumatics;

// import org.team2168.commands.auto.selector.AutoStartLeft1Cube;
// import org.team2168.commands.auto.selector.AutoStartLeft2Cube;

// import org.team2168.commands.auto.selector.AutoStartLeft2CubeSuperDooperPooper;
// import org.team2168.commands.auto.selector.AutoStartLeft3CubeNotSafe;
// import org.team2168.commands.auto.selector.AutoStartLeft3CubeSafe;
// import org.team2168.commands.auto.selector.AutoStartRight2CubeSafe;
// import org.team2168.commands.lights.AutoWithoutCube;
// import org.team2168.commands.lights.SuckPattern;
// import org.team2168.commands.lights.TeleopWithoutCube;
// import org.team2168.commands.pneumatics.*;
import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot
{
	//Digital Jumper to Identify if this is practice bot or comp bot
	private static DigitalInput practiceBot;

	//Operator Interface
	public static OI oi;
	
	//Subsystems
	// public static CubeIntakeWheels cubeIntakeWheels;
	// public static CubeIntakeGripper cubeIntakeGripper;
	public static Drivetrain drivetrain;
	// public static DrivetrainShifter drivetrainShifter;
	// public static Lift lift;
	// public static LiftRatchetShifter liftRatchetShifter;
	// public static LiftShifter liftShifter;
	// public static PivotHardStop flipperyFloopyFlupy;
	// public static IntakePivotPiston intakePivotPiston;
	// public static Winch winch;
	// //public static Platform platform;
	public static Pneumatics pneumatics;
	public static I2C i2c;
	byte[] toSend = new byte[1];
	
	

	// Variables for initializing and calibrating the Gyro
	static boolean autoMode;
	private static boolean matchStarted = false;
	public static int gyroReinits;
	private double lastAngle;
	private Debouncer gyroDriftDetector = new Debouncer(1.0);
	public static boolean gyroCalibrating = false;
	private boolean lastGyroCalibrating = false;
	private double curAngle = 0.0;

	public static String test = "Bye";
	public static boolean variable = true;
	
	//Driverstation Instance
	public static DriverStation driverstation;
	
	//PDP Instance
	public static PowerDistribution pdp;
	
	//Autonomous Chooser
    static Command autonomousCommand;
    public static SendableChooser<Command> autoChooser;
    
    //Driver Joystick Chooser
    static int controlStyle;
    public static SendableChooser<Number> controlStyleChooser;
    
  //Driver Joystick Chooser
    static int autoPriority;
    public static SendableChooser<Number> autoPriorityChooser;
    

	double runTime = Timer.getFPGATimestamp();
    
    //Turn on TX1
    //TX1TurnON tx1;
    
    //Global Position Tracking Class
   // public static DrivetrainIMUGlobalPosition dtIMU;
	
    //Variable to track blue alliance vs red alliance
    private static boolean blueAlliance = false;
    
    public static OneDimensionalMotionProfiling motion;
    
    
    public static double[] leftVelPathQuintic;
    public static double[] rightVelPathQuintic;
    public static double[] leftPosPathQuintic;
    public static double[] rightPosPathQuintic;
    public static double[] headingQuintic;
    
    public static double[] leftVelPathQuintic2;
    public static double[] rightVelPathQuintic2;
    public static double[] leftPosPathQuintic2;
    public static double[] rightPosPathQuintic2;
    public static double[] headingQuintic2;
    
    public static double[] leftVelPathQuintic3;
    public static double[] rightVelPathQuintic3;
    public static double[] leftPosPathQuintic3;
    public static double[] rightPosPathQuintic3;
    public static double[] headingQuintic3;
    
    public static double[] leftVelPathQuintic4;
    public static double[] rightVelPathQuintic4;
    public static double[] leftPosPathQuintic4;
    public static double[] rightPosPathQuintic4;
    public static double[] headingQuintic4;
 
    
    public static double[] leftVelPathQuintic5;
    public static double[] rightVelPathQuintic5;
    public static double[] leftPosPathQuintic5;
    public static double[] rightPosPathQuintic5;
    public static double[] headingQuintic5;
    
    public static double[] leftVelPathQuintic6;
    public static double[] rightVelPathQuintic6;
    public static double[] leftPosPathQuintic6;
    public static double[] rightPosPathQuintic6;
    public static double[] headingQuintic6;
    
    public static double[] leftVelPathQuintic7;
    public static double[] rightVelPathQuintic7;
    public static double[] leftPosPathQuintic7;
    public static double[] rightPosPathQuintic7;
    public static double[] headingQuintic7;
    
    public static double[] leftVelPathQuintic8;
    public static double[] rightVelPathQuintic8;
    public static double[] leftPosPathQuintic8;
    public static double[] rightPosPathQuintic8;
    public static double[] headingQuintic8;
    
    public static double[] leftVelPathQuintic9;
    public static double[] rightVelPathQuintic9;
    public static double[] leftPosPathQuintic9;
    public static double[] rightPosPathQuintic9;
    public static double[] headingQuintic9;
    
    public static double[] leftVelPathQuintic10;
    public static double[] rightVelPathQuintic10;
    public static double[] leftPosPathQuintic10;
    public static double[] rightPosPathQuintic10;
    public static double[] headingQuintic10;
    
    public static double[] leftVelPathQuintic11;
    public static double[] rightVelPathQuintic11;
    public static double[] leftPosPathQuintic11;
    public static double[] rightPosPathQuintic11;
    public static double[] headingQuintic11;
    
    public static double[] leftVelPathQuintic12;
    public static double[] rightVelPathQuintic12;
    public static double[] leftPosPathQuintic12;
    public static double[] rightPosPathQuintic12;
    public static double[] headingQuintic12;
    
    
    public static double[] leftVelPathQuintic13;
    public static double[] rightVelPathQuintic13;
    public static double[] leftPosPathQuintic13;
    public static double[] rightPosPathQuintic13;
    public static double[] headingQuintic13;
    
    public static double[] leftPosPathQuintic14;
    public static double[] rightPosPathQuintic14;
    public static double[] leftVelPathQuintic14;
    public static double[] rightVelPathQuintic14;
    public static double[] headingQuintic14;
    
 
    
    
    public static String gameData = "N A";

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		try
		{
		// this.setPeriod(RobotMap.MAIN_PERIOD_S);

		//Stop all WPILib 2018 Telementry
		LiveWindow.disableAllTelemetry();
		
		ConsolePrinter.init();
		ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS);

		practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);

				
		// Instantiate the subsystems
		// intakePivotPiston = IntakePivotPiston.getInstance();
		// cubeIntakeGripper = CubeIntakeGripper.getInstance();
		// cubeIntakeWheels =  CubeIntakeWheels.getInstance();
		drivetrain = Drivetrain.getInstance();
		// drivetrainShifter = DrivetrainShifter.getInstance();
		// lift = Lift.GetInstance();
		// liftShifter = LiftShifter.GetInstance();
		// liftRatchetShifter = LiftRatchetShifter.GetInstance();
		// //platform = platform.getInstance();
		pneumatics = Pneumatics.getInstance();
		// //scissorLift = ScissorLift.getInstance();
		// flipperyFloopyFlupy = PivotHardStop.getInstance();
		// winch = Winch.GetInstance();
		i2c = new I2C(I2C.Port.kOnboard, 8);
		
//		motion = new OneDimensionalMotionProfiling(15);
//		for(int i=0; i<motion.getVelArray().length; i++)
//			System.out.println(motion.getVelArray()[i]);
//		//Start Thread Only After Every Other Class is Loaded. 
		
		
		double[][] waypointPath = new double[][]{
			{1, 26, 0}, //For left switch & right scale from left side
			{11.5, 27.0, 0},
			{13.0, 25.5, -Math.PI/2 + 0.0001}		
			
	};

		QuinticTrajectory quinticPath= new QuinticTrajectory("path1", waypointPath);
		
		this.leftVelPathQuintic = quinticPath.getLeftVel();
		this.rightVelPathQuintic = quinticPath.getRightVel();
		this.headingQuintic = quinticPath.getHeadingDeg();
		
		
		double[][] waypointPath2 = new double[][]{
			{14.5, 23.5, Math.PI/2},
			{21, 27.0, 0+0.0001},
			{22.5, 27.0, 0}
			
			//{27, 20, 0}	

	};
		

		QuinticTrajectory quinticPath2= new QuinticTrajectory("path2",waypointPath2);
		this.leftVelPathQuintic2 = quinticPath2.getLeftVel();
		this.rightVelPathQuintic2 = quinticPath2.getRightVel();
		this.headingQuintic2 = quinticPath2.getHeadingDeg();
		
	
	    double[][] waypointPath3 = new double[][]
      {
	    	{1, 15.5, 0}, //Right switch Path
			{2, 15.5, 0},
			{9.5, 11.5, 0} //need to add 1.5 to 12.6 //for 4th match  
	};


		QuinticTrajectory quinticPath3 = new QuinticTrajectory("path3",waypointPath3);
		
		this.leftVelPathQuintic3 = quinticPath3.getLeftVel();
		this.rightVelPathQuintic3 = quinticPath3.getRightVel();
		this.headingQuintic3 = quinticPath3.getHeadingDeg();
		
	    double[][] waypointPath4 = new double[][]{
	    	//{5, 17, Math.PI/2}, //For Right switch from center 
			//{5, 19, Math.PI/2},
			//{8.5, 23, Math.PI/2},
			//{8.5, 24, Math.PI/2}
			
	    	{1, 15.5, 0}, //Right switch Path
			{2, 15.5, 0},
			{10.5, 20.3, 0} 
		};


		QuinticTrajectory quinticPath4 = new QuinticTrajectory("path4",waypointPath4);
		
		this.leftVelPathQuintic4 = quinticPath4.getLeftVel();
		this.rightVelPathQuintic4 = quinticPath4.getRightVel();
		this.headingQuintic4 = quinticPath4.getHeadingDeg();
		
		//Start Thread Only After Every Other Class is Loaded. 
		
	    double[][] waypointPath5 = new double[][]{
	    	//{5, 17, Math.PI/2}, //For Right switch from center 
			//{5, 19, Math.PI/2},
			//{8.5, 23, Math.PI/2},
			//{8.5, 25, Math.PI/2}
			
//			{10, 24, 0},
//			{23, 24, 0},
//			{27, 20, -Math.PI/2+0.0001},
//			{27, 8, -Math.PI/2+0.0001},
//			{29,5, 0}
			

	    	{10, 24, 0},
			{22.5, 24, 0},
			{26.0, 20, -Math.PI/2+0.0001},
			{26.0, 17, -Math.PI/2+0.0001}, //end of comp
			{26.0, 13, -Math.PI/2+0.0001},
			{26.0, 8.6, -Math.PI/2+0.0001},
			{27.2, 6.9, 0}	
		};
		

		QuinticTrajectory quinticPath5 = new QuinticTrajectory("path5",waypointPath5);
		this.leftVelPathQuintic5 = quinticPath5.getLeftVel();
		this.rightVelPathQuintic5 = quinticPath5.getRightVel();
		this.headingQuintic5 = quinticPath5.getHeadingDeg();

		double[][] waypointPath6 = new double[][]{
//			{2, 26, 0},
//			{15.5, 26, 0},
//			{19.5, 25.5, -Math.PI/6}
			
//			{2, 26, 0},
//			{18.5, 27.5, 0},
//			{23.5, 26, -Math.PI/2.5}
			
			
			//{2, 26, 0},  //OG Path
			//{14.3, 27, 0},
			//{19.1, 26.5, -Math.PI/5}
			
			{2, 26.5, 0}, //crazy path
			{17.0, 26.5, 0},
			{19.6, 25.5, -0.349} //works with 19.3 on practice bot
//			{2, 26, 0},
//			{17.5, 26, 0},
//			{21.5, 26, -Math.PI/3.5}
		};
		
		QuinticTrajectory quinticPath6 = new QuinticTrajectory("path6",waypointPath6);
		quinticPath6.calculate();
		
		this.leftVelPathQuintic6 = quinticPath6.getLeftVel();
		this.rightVelPathQuintic6 = quinticPath6.getRightVel();
		this.headingQuintic6 = quinticPath6.getHeadingDeg();

		double[][] waypointPath7 = new double [][] {
			{2, 26.5, 0}, //crazy path
			{15.0, 26.5, 0},
			{18.6, 27.5, 0.349} //works with 19.3 on practice bot
			
		};
		
		QuinticTrajectory quinticPath7 = new QuinticTrajectory("path7",waypointPath7);
		quinticPath7.calculate();
		
		this.leftVelPathQuintic7 = quinticPath7.getLeftVel();
		this.rightVelPathQuintic7 = quinticPath7.getRightVel();
		this.headingQuintic7 = quinticPath7.getHeadingDeg();
		
		double[][] waypointPath8 = new double[][] {
			{1, 26, 0}, //For right switch 
			{11.5, 25.0, 0}, //25
			{13.0, 26.5, Math.PI/2 + 0.0001} //26.5
		};
		
		QuinticTrajectory quinticPath8 = new QuinticTrajectory("path8",waypointPath8);
		this.leftVelPathQuintic8 = quinticPath8.getLeftVel();
		this.rightVelPathQuintic8 = quinticPath8.getRightVel();
		this.headingQuintic8 = quinticPath8.getHeadingDeg();
		
		double[][] waypointPath9 = new double[][] {
			{14.5, 8.5, Math.PI/2}, // 8.5
			{21, 5.0, 0+0.0001}, //5
			{22.5, 5.0, 0} //5
		};
		
		QuinticTrajectory quinticPath9 = new QuinticTrajectory("path9",waypointPath9);
		this.leftVelPathQuintic9 = quinticPath9.getLeftVel();
		this.rightVelPathQuintic9 = quinticPath9.getRightVel();
		this.headingQuintic9 = quinticPath9.getHeadingDeg();
		
		double[][] waypointPath10 = new double[][] {
			{10, 8, 0},
			{22.0, 8, 0},
			{25.0, 12, -Math.PI/2+0.0001},
			{25.0, 15, -Math.PI/2+0.0001}, //end
			{25.0, 19, -Math.PI/2+0.0001},
			{25.0, 24.0, -Math.PI/2+0.0001},
			{27.0, 26.0, 0}
		};
		QuinticTrajectory quinticPath10 = new QuinticTrajectory("path10",waypointPath10);
		this.leftVelPathQuintic10 = quinticPath10.getLeftVel();
		this.rightVelPathQuintic10 = quinticPath10.getRightVel();
		this.headingQuintic10 = quinticPath10.getHeadingDeg();
		
		double[][] waypointPath11 = new double[][] {
			{10, 24, 0},
			{22.5, 24, 0},
			{26.0, 20, -Math.PI/2+0.0001},
			{26.0, 17, -Math.PI/2+0.0001} //end of comp
			//{25.5, 13, -Math.PI/2+0.0001},
			//{25.5, 8.0, -Math.PI/2+0.0001},
			//{27.5, 6.0, 0}
		};
		QuinticTrajectory quinticPath11 = new QuinticTrajectory("path11",waypointPath11);
		this.leftVelPathQuintic11 = quinticPath11.getLeftVel();
		this.rightVelPathQuintic11 = quinticPath11.getRightVel();
		this.headingQuintic11 = quinticPath11.getHeadingDeg();
		
		double[][] waypointPath12= new double[][] {
			{10, 8, 0},
			{22.0, 8, 0},
			{25.0, 12, -Math.PI/2+0.0001}
			//{25.0, 15, -Math.PI/2+0.0001}, //end
			//{25.0, 19, -Math.PI/2+0.0001},
			//{25.0, 24.0, -Math.PI/2+0.0001},
			//{27.0, 26.0, 0}
		};
		QuinticTrajectory quinticPath12 = new QuinticTrajectory("path12",waypointPath12);
		this.leftVelPathQuintic12 = quinticPath12.getLeftVel();
		this.rightVelPathQuintic12 = quinticPath12.getRightVel();
		this.headingQuintic12 = quinticPath12.getHeadingDeg();
		
		double[][] waypointPath13= new double[][] {
			{17.0, 26.5, 0},
			{19.6, 25.5, -0.349} //works with 19.3 on practice bot
			//{25.0, 15, -Math.PI/2+0.0001}, //end
			//{25.0, 19, -Math.PI/2+0.0001},
			//{25.0, 24.0, -Math.PI/2+0.0001},
			//{27.0, 26.0, 0}
		};
		QuinticTrajectory quinticPath13 = new QuinticTrajectory("path13",waypointPath13);
		this.leftVelPathQuintic13 = quinticPath13.getLeftVel();
		this.rightVelPathQuintic13 = quinticPath13.getRightVel();
		this.headingQuintic13 = quinticPath13.getHeadingDeg();
		
		
		double[][] waypointPath14= new double[][] {
			{0.0, 26.5, 0},
			{5.0, 26.5, 0} //works with 19.3 on practice bot
			//{25.0, 15, -Math.PI/2+0.0001}, //end
			//{25.0, 19, -Math.PI/2+0.0001},
			//{25.0, 24.0, -Math.PI/2+0.0001},
			//{27.0, 26.0, 0}
		};
		QuinticTrajectory quinticPath14 = new QuinticTrajectory("path14",waypointPath14);
		this.leftPosPathQuintic14 = quinticPath14.getLeftPos();
		this.rightPosPathQuintic14 = quinticPath14.getRightPos();
		this.leftVelPathQuintic14 = quinticPath14.getLeftVel();
		this.rightVelPathQuintic14 = quinticPath14.getRightVel();
		this.headingQuintic14 = quinticPath14.getHeadingDeg();
		
		
		
		
		//Start Operator Interface
		oi = OI.getInstance();

		// enable compressor
		new StartCompressor();

		//Initialize Autonomous Selector Choices
		autoSelectInit();
		
		//Initialize Control Selector Choices
		controlStyleSelectInit();
		
		//Initialize Control Selector Choices
		AutoPrioritySelectInit();
				

		pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
		pdp.startThread();

		//tx1 = new TX1TurnON(RobotMap.PDPThreadPeriod);
		//tx1.startThread();

		//dtIMU = new DrivetrainIMUGlobalPosition(RobotMap.PDPThreadPeriod);
		//dtIMU.startThread();

		drivetrain.calibrateGyro();
		driverstation = DriverStation.getInstance();


		
		

		
		
		
		
		
		
		
		ConsolePrinter.putSendable("Control Style Chooser", () -> {return Robot.controlStyleChooser;}, true, false);
		ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);
		ConsolePrinter.putSendable("Priority Mode Chooser", () -> {return Robot.autoPriorityChooser;}, true, false);
		ConsolePrinter.putString("AutoName", () -> {return Robot.getAutoName();}, true, false);
		ConsolePrinter.putString("Control Style Name", () -> {return Robot.getControlStyleName();}, true, false);
		ConsolePrinter.putString("Auto Priority Name", () -> {return Robot.getAutoPriority();}, true, false);
		ConsolePrinter.putNumber("gameClock", () -> {return driverstation.getMatchTime();}, true, false);
		ConsolePrinter.putNumber("Robot Pressure", () -> {return Robot.pneumatics.getPSI();}, true, false);
		ConsolePrinter.putBoolean("Is Practice Bot", () -> {return isPracticeRobot();}, true, false);
		ConsolePrinter.putString("Switch_Scale_Switch orientation", () -> {return driverstation.getGameSpecificMessage();}, true, false); //Ill show you de wei

		
		
		
		

		ConsolePrinter.startThread();
		System.out.println("************Robot Done Loading Successfully**********");
		}
		catch (Throwable throwable) 
		{
		      Throwable cause = throwable.getCause();
		      if (cause != null) 
		        throwable = cause;
		      
		      System.err.println("Bad things occured, testing using our own stach trace catch");
		      System.err.println("Implement Logging function here");
		      System.err.flush();
		      
		      //Show Stack Trace on Driverstration like before
		      DriverStation.reportError("Unhandled exception instantiating robot" 
		              + throwable.toString(), throwable.getStackTrace());
		          DriverStation.reportWarning("Robots should not quit, but yours did!", false);
		          DriverStation.reportError("Could not instantiate robot!", false);
		          System.exit(1);
		      return;
		}
		
		
		LiveWindow.disableAllTelemetry();
		
	}
	
    /************************************************************
    *
    * 				Robot State Machine
    * 
    ************************************************************/
	
	
	//
	/**
	 * This method is called once each time the robot enters Disabled mode. You can
	 * use it to reset any subsystem information you want to clear when the robot is
	 * disabled.
	 */
	public void disabledInit() 
	{
		autoMode = false;
		matchStarted = false;
		Robot.i2c.write(8, 4);
		
		//If we are not in a match allow Gyro to be recalibrated in Disabled even if a previous 
		//calibration was performed, we disable this in a match so that if we ever die in a match,
		//we don't try to recalibrate a moving robot. 
		if(driverstation.isFMSAttached())
			drivetrain.startGyroCalibrating();
		
		drivetrain.calibrateGyro();
		callArduino();
		//i2c.write(8, 0);
		
		
	}

	public void disabledPeriodic() 
	{

		//Keep track of Gunstyle Controller Variables
		
		callArduino();
		getControlStyleInt();
		controlStyle = (int) controlStyleChooser.getSelected();
		autoPriority = (int) autoPriorityChooser.getSelected();
		autonomousCommand = (Command) autoChooser.getSelected();

		//Continuously get field data
		getFieldData();
		
		// Kill all active commands
		Scheduler.getInstance().run();

	}
	

    public void autonomousInit() 
    {
    	autoMode = true;
    	
    	//get field data one last time
    	getFieldData();	
		matchStarted = true;
		drivetrain.stopGyroCalibrating();
		drivetrain.resetGyro();
		
		
		autonomousCommand = (Command) autoChooser.getSelected();
    	
        // schedule the autonomous command
        if (autonomousCommand != null) 
        	autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	autoMode = true;
        Scheduler.getInstance().run();
 //       Scheduler.getInstance().add(new AutoWithoutCube());
        
    }	
	
    /**
     * This function called prior to robot entering Teleop Mode
     */
	public void teleopInit() 
	{
		//callArduino();
	    	autoMode = false;
	    	//Robot.i2c.write(8, 97);
			matchStarted = true;
			drivetrain.stopGyroCalibrating();
	    	
	    	// This makes sure that the autonomous stops running when
	        // teleop starts running. If you want the autonomous to 
	        // continue until interrupted by another command, remove
	        // this line or comment it out.
	        if (autonomousCommand != null) autonomousCommand.cancel();
	
	    	// Select the control style
	        controlStyle = (int) controlStyleChooser.getSelected();
	        
	        runTime = Timer.getFPGATimestamp();
	        // Scheduler.getInstance().add(new TeleopWithoutCube());     
	  }
	    

	    /**
	     * This function is called periodically during operator control
	     */
	    public void teleopPeriodic() {
	    	
	        SmartDashboard.putNumber("TeleopLoopTime", Timer.getFPGATimestamp()-runTime);
	        runTime = Timer.getFPGATimestamp();
	    
	    	autoMode = false;
	        Scheduler.getInstance().run();
	        
	        controlStyle = (int) controlStyleChooser.getSelected();
	//        updateLights();
	        callArduino();
	        //Robot.i2c.write(8, 97);
	        
	        	
	        
	        

	    }

	    
	    /************************************************************
	     *
	     * 			HELPER FUNCTIONS FOR ENTIRE ROBOT
	     * 
	     ************************************************************/
	
	    

		/**
		 * Get the name of an autonomous mode command.
		 * 
		 * @return the name of the auto command.
		 */
		public static String getAutoName() {
			if (autonomousCommand != null) {
				return autonomousCommand.getName();
			} else {
				return "None";
			}
		}

		
		
		/**
		 * Get the name of auto priority
		 * 
		 * @return the name of auto priority
		 */
		public static String getAutoPriority() {
			String retVal = "";

			switch (autoPriority) {
			case 0:
				retVal = "Switch";
				break;
			case 1:
				retVal = "Scale";
				break;
			default:
				retVal = "Invalid Auto Priority";
			}

			return retVal;
		}

		/**
		 * Adds auto priorities to the selector
		 */
		public void AutoPrioritySelectInit() {
			autoPriorityChooser = new SendableChooser<>();
			autoPriorityChooser.addObject("Switch", 0);
			autoPriorityChooser.addDefault("Scale", 1);
		}

		public static int getAutoPriorityInt() {
			return (int) autoPriorityChooser.getSelected();
		}

		/**
		 * Get the name of an control style.
		 * 
		 * @return the name of the control style.
		 */
		public static String getControlStyleName() {
			String retVal = "";

			switch (controlStyle) {
			case 0:
				retVal = "Tank Drive";
				break;
			case 1:
				retVal = "Gun Style";
				break;
			case 2:
				retVal = "Arcade Drive";
				break;
			case 3:
				retVal = "GTA Drive";
				break;
			default:
				retVal = "Invalid Control Style";
			}

			return retVal;
		}

		/**
		 * Adds control styles to the selector
		 */
		public void controlStyleSelectInit() {
			controlStyleChooser = new SendableChooser<>();
			controlStyleChooser.addObject("Tank Drive", 0);
			controlStyleChooser.addDefault("Gun Style Controller", 1);
			controlStyleChooser.addObject("Arcade Drive", 2);
			controlStyleChooser.addObject("GTA Drive", 3);
		}

		public static int getControlStyleInt() {
			return (int) controlStyleChooser.getSelected();
		}

		
		
		
		/**
		 * Adds the autos to the selector
		 */
		public void autoSelectInit() {
			autoChooser = new SendableChooser<Command>();
			// autoChooser.addDefault("Drive Straight", new DriveStraight(8.0));
			// autoChooser.addObject("Do Nothing", new DoNothing());
	        // autoChooser.addObject("Center Auto 2 Cube", new AutoStartCenter2Cube());	        
	        //autoChooser.addObject("Left Auto 1 Cube", new AutoStartLeft1Cube());
	        //autoChooser.addObject("Left Auto 2 Cube", new AutoStartLeft2Cube());
			//autoChooser.addObject("Left Auto 3 Dangerous", new AutoStartLeft2CubeSuperDooperPooper());
			// autoChooser.addObject("Right Auto 2 Cube Safe", new AutoStartRight2CubeSafe());
			// autoChooser.addObject("Left Auto 3 Cube Safe", new AutoStartLeft3CubeSafe());
			// autoChooser.addObject("Left Auto 3 Cube Very very (maybe) Safe", new AutoStartLeft3CubeNotSafe());
			// autoChooser.addObject("Dont try this at home", new TestAuto());
			// autoChooser.addObject("Dont run me", new NeverRunMe());
//			autoChooser.addObject("Dont cross me from left", new DriveToRightScaleFromLeft());
//			autoChooser.addObject("Dont cross me from right", new DriveToLeftScaleFromRightSide());
			
			autoChooser.addObject("2 Feet", new DriveStraight2Feet());
			autoChooser.addObject("4 Feet", new DriveStraight4Feet());
			autoChooser.addObject("6 Feet", new DriveStraight6Feet());
			autoChooser.addObject("8 Feet", new DriveStraight8Feet());
			autoChooser.addObject("10 Feet", new DriveStraight10Feet());

		}

		/**
		 *
		 * @return true if the robot is in auto mode
		 */
		public static boolean isAutoMode() {
			return autoMode;

		}

		
		
		/**
		 * Method which checks to see if gyro drifts and resets the gyro. Call this in a
		 * loop.
		 */
		private void gyroReinit() {
			// Check to see if the gyro is drifting, if it is re-initialize it.
			// Thanks FRC254 for orig. idea.
			curAngle = drivetrain.getHeading();
			gyroCalibrating = drivetrain.isGyroCalibrating();

			if (lastGyroCalibrating && !gyroCalibrating) {
				// if we've just finished calibrating the gyro, reset
				gyroDriftDetector.reset();
				curAngle = drivetrain.getHeading();
				System.out.println("Finished auto-reinit gyro");
			} else if (gyroDriftDetector.update(Math.abs(curAngle - lastAngle) > (0.75 / 50.0)) && !matchStarted
					&& !gyroCalibrating) {
				// && gyroReinits < 3) {
				gyroReinits++;
				System.out.println("!!! Sensed drift, about to auto-reinit gyro (" + gyroReinits + ")");
				drivetrain.calibrateGyro();
			}

			lastAngle = curAngle;
			lastGyroCalibrating = gyroCalibrating;

		}
	    


	/**
	 * Returns the status of DIO pin 24
	 *
	 * @return true if this is the practice robot
	 */
	public static boolean isPracticeRobot() {
		return !practiceBot.get();

	}


	public static boolean onBlueAlliance() {
		return driverstation.getAlliance() == DriverStation.Alliance.Blue;

	}
	
	
	// This function is called periodically during test mode
	public void testPeriodic() {
		
		LiveWindow.run();
		
	}
	
	public String getFieldData()
	{
		String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
		
		if(gameMessage.length()==3 && this.gameData != null)
		{
			gameData = gameMessage;
		}
		else
			gameData = "N A";
		return gameData;
	}
	
	// private void updateLights()  {                          
	// if(!Robot.autoMode) {
	// if (cubeIntakeWheels.isCubePresent()) {
    // 	cubeIntakeWheels.setLights(1);
	//     Robot.i2c.write(8, 1);}
    // else 
    // 	cubeIntakeWheels.setLights(-1);
	// }}
	
	private void callArduino() {
		//toSend[0] =  74;
		//i2c.write(8, 'a');
		//System.out.println(i2c.write(8, 'a'));
	}
	
}
