package org.team2168.robot;

import org.team2168.PID.trajectory.OneDimensionalMotionProfiling;
import org.team2168.PID.trajectory.QuinticTrajectory;
import org.team2168.commands.pneumatics.StartCompressor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.DrivetrainShifter;
import org.team2168.subsystems.Pneumatics;
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
  private static DigitalInput canBot;

	//Operator Interface
	public static OI oi;
	
	//Subsystems
	public static Drivetrain drivetrain;
  public static Pneumatics pneumatics;
  public static DrivetrainShifter drivetrainShifter;
	

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
    public static double[] headingQuintic;
    
    public static double[] leftVelPathQuintic2;
    public static double[] rightVelPathQuintic2;
    public static double[] headingQuintic2;
    
    public static double[] leftVelPathQuintic3;
    public static double[] rightVelPathQuintic3;
    public static double[] headingQuintic3;
    
    public static double[] leftVelPathQuintic4;
    public static double[] rightVelPathQuintic4;
    public static double[] headingQuintic4;
 
    
    public static double[] leftVelPathQuintic5;
    public static double[] rightVelPathQuintic5;
    public static double[] headingQuintic5;
    
    public static double[] leftVelPathQuintic6;
    public static double[] rightVelPathQuintic6;
    public static double[] headingQuintic6;
    
    public static double[] leftVelPathQuintic7;
    public static double[] rightVelPathQuintic7;
    public static double[] headingQuintic7;
    
    public static double[] leftVelPathQuintic8;
    public static double[] rightVelPathQuintic8;
    public static double[] headingQuintic8;
    
    public static double[] leftVelPathQuintic9;
    public static double[] rightVelPathQuintic9;
    public static double[] headingQuintic9;
    
    public static double[] leftVelPathQuintic10;
    public static double[] rightVelPathQuintic10;
    public static double[] headingQuintic10;
    
    public static double[] leftVelPathQuintic11;
    public static double[] rightVelPathQuintic11;
    public static double[] headingQuintic11;
    
    public static double[] leftVelPathQuintic12;
    public static double[] rightVelPathQuintic12;
    public static double[] headingQuintic12;
    
    public static double[] leftVelPathQuintic13;
    public static double[] rightVelPathQuintic13;
    public static double[] headingQuintic13;
    
 
    
    
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
		

		//Stop all WPILib 2018 Telementry
		LiveWindow.disableAllTelemetry();
		
		ConsolePrinter.init();
		ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS);

    practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);
    canBot = new DigitalInput(RobotMap.CAN_BOT_JUMPER);

			
    drivetrain = Drivetrain.getInstance();
    drivetrainShifter = DrivetrainShifter.getInstance();
    pneumatics = Pneumatics.getInstance();
		i2c = new I2C(I2C.Port.kOnboard, 8);
		

		double[][] waypointPath = new double[][]{
			{1, 26, 0}, //For left switch & right scale from left side
			{11.5, 27.0, 0},
			{13.0, 25.5, -Math.PI/2 + 0.0001}		
			
	};

		QuinticTrajectory quinticPath= new QuinticTrajectory("path1", waypointPath);
		
		this.leftVelPathQuintic = quinticPath.getLeftVel();
		this.rightVelPathQuintic = quinticPath.getRightVel();
		this.headingQuintic = quinticPath.getHeadingDeg();
		
		
	
		
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
        //Scheduler.getInstance().add(new AutoWithoutCube());
        
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
	        //Scheduler.getInstance().add(new TeleopWithoutCube());     
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
	        updateLights();
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
	       
			// autoChooser.addObject("Center Auto 3 Cube", new AutoStartCenter3Cube());	        
	       
			// autoChooser.addObject("Left Auto 3 Cube Safe", new AutoStartLeft3CubeSafe());
			// autoChooser.addObject("Left Auto 3 Cube not Safe", new AutoStartLeft3CubeNotSafe());
			// autoChooser.addObject("Left Auto Simple", new AutoStartLeftSimple());
			
			// autoChooser.addObject("Right Auto 3 Cube Safe", new AutoStartRight3CubeSafe());
			// autoChooser.addObject("Right Auto 3 Cube not Safe", new AutoStartRight3CubeNotSafe());
			// autoChooser.addObject("Right Auto Simple", new AutoStartRightSimple());


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
  
  public static boolean isCanRobot() {
  
		return !canBot.get();

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
	
	private void updateLights()  {                          
	// if(!Robot.autoMode) {
	// if (cubeIntakeWheels.isCubePresent()) {
  //   	cubeIntakeWheels.setLights(1);
	//     Robot.i2c.write(8, 1);}
  //   else 
  //   	cubeIntakeWheels.setLights(-1);
	}
	
	private void callArduino() {
		//toSend[0] =  74;
		//i2c.write(8, 'a');
		//System.out.println(i2c.write(8, 'a'));
	}
	
}
