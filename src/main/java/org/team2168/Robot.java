/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.subsystems.CargoIntake;
import org.team2168.subsystems.CargoPunch;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.DrivetrainStingerShifter;
import org.team2168.subsystems.HatchProbePistons;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.LiftBrake;
import org.team2168.subsystems.MonkeyBar;
import org.team2168.subsystems.HatchProbePivotBrake;
import org.team2168.subsystems.HatchProbePivot;
import org.team2168.subsystems.Stinger;
import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Digital Jumper to Identify if this is practice bot or comp bot
  private static DigitalInput practiceBot;

  //Digital Jumper to Identify if this uses a CAN drivetrain
  private static DigitalInput canDrivetrain;
  
  //Operator Interface
  public static OI oi;

  //Subsystems
  public static CargoIntake cargointake;
  public static Drivetrain drivetrain;
  public static DrivetrainStingerShifter drivetrainStingerShifter;
  public static Lift lift;
  public static LiftBrake liftHardStop;
  public static HatchProbePivot plungerArmPivot;
  public static HatchProbePivotBrake plungerArmBrake;
  public static HatchProbePistons hatchPlunger;
  public static HatchFloorIntake floorHatchIntake;
  public static Stinger stinger;
  public static MonkeyBar monkeyBar;
  public static CargoPunch cargoPunch;

  // Variables for initializing and calibrating the Gyro
  static boolean autoMode;
  private static boolean matchStarted = false;
  public static int gyroReinits;
  private double lastAngle;
  private Debouncer gyroDriftDetector = new Debouncer(1.0);
  public static boolean gyroCalibrating = false;
  private boolean lastGyroCalibrating = false;
  private double curAngle = 0.0;

  //PDP Instance
  public static PowerDistribution pdp;

  //Driverstation Instance
	public static DriverStation driverstation;

  //Driver Joystick Chooser
  static int controlStyle;
  static Command autonomousCommand;
  public static SendableChooser<Command> autoChooser;
  public static SendableChooser<Number> controlStyleChooser;

  //Keep track of time
  double runTime = Timer.getFPGATimestamp();

  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
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
      canDrivetrain = new DigitalInput(RobotMap.CAN_DRIVETRAIN_JUMPER);

      //Instantiate the subsystems
      drivetrain = Drivetrain.getInstance();
      drivetrainStingerShifter = DrivetrainStingerShifter.getInstance();
      lift = Lift.getInstance();
      liftHardStop = LiftBrake.getInstance();
      plungerArmPivot = HatchProbePivot.getInstance();
      plungerArmBrake = HatchProbePivotBrake.getInstance();
      hatchPlunger = new HatchProbePistons();
      floorHatchIntake = HatchFloorIntake.getInstance();
      monkeyBar = new MonkeyBar.getInstance();
      cargoPunch = CargoPunch.getInstance();

      i2c = new I2C(I2C.Port.kOnboard, 8);

      drivetrain.calibrateGyro();
      driverstation = DriverStation.getInstance();

      //Starting PDP
      pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
      pdp.startThread();

      //Start Operator Interface
      oi = OI.getInstance();
      
		  // enable compressor
		  new StartCompressor();

		  //Initialize Autonomous Selector Choices
		  autoSelectInit();
		  controlStyleSelectInit();
      

      ConsolePrinter.putSendable("Control Style Chooser", () -> {return Robot.controlStyleChooser;}, true, false);
      ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);
      ConsolePrinter.putSendable("Throttle Vibe Chooser", () -> {return Robot.throttleVibeChooser;}, true, false);
      ConsolePrinter.putString("AutoName", () -> {return Robot.getAutoName();}, true, false);
      ConsolePrinter.putString("Control Style Name", () -> {return Robot.getControlStyleName();}, true, false);
      ConsolePrinter.putNumber("gameClock", () -> {return driverstation.getMatchTime();}, true, false);
      ConsolePrinter.putNumber("Robot Pressure", () -> {return Robot.pneumatics.getPSI();}, true, false);
      ConsolePrinter.putBoolean("Is Practice Bot", () -> {return isPracticeRobot();}, true, false);
      ConsolePrinter.putString("Switch_Scale_Switch orientation", () -> {return driverstation.getGameSpecificMessage();}, true, false); //Ill show you de wei

      ConsolePrinter.startThread();
      System.out.println("Robot Initialization Complete!!");
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

  }

    /************************************************************
    *
    * 				Robot State Machine
    * 
    ************************************************************/

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

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
    
    //If we are not in a match allow Gyro to be recalibrated in Disabled even if a previous 
    //calibration was performed, we disable this in a match so that if we ever die in a match,
    //we don't try to recalibrate a moving robot. 
    if(driverstation.isFMSAttached())
    	drivetrain.startGyroCalibrating();
    
    drivetrain.calibrateGyro();
    
    
  }

  public void disabledPeriodic() 
  {

    //Keep track of Gunstyle Controller Variables
    
    //callArduino();
    getControlStyleInt();
    controlStyle = (int) controlStyleChooser.getSelected();
    // autoPriority = (int) autoPriorityChooser.getSelected();
    // autonomousCommand = (Command) autoChooser.getSelected();
    
    // Kill all active commands
    Scheduler.getInstance().run();

  }


    public void autonomousInit() 
    {
      autoMode = true;
      
    matchStarted = true;
    drivetrain.stopGyroCalibrating();
    drivetrain.resetGyro();
    
    
    // autonomousCommand = (Command) autoChooser.getSelected();
      
    //     // schedule the autonomous command
    //     if (autonomousCommand != null) 
    //       autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
      autoMode = true;
        Scheduler.getInstance().run();
      //  Scheduler.getInstance().add(new AutoWithoutCube());
        
    }	

    /**
     * This function called prior to robot entering Teleop Mode
     */
  public void teleopInit() 
  {
    //callArduino();
        autoMode = false;
      matchStarted = true;
      drivetrain.stopGyroCalibrating();
        
        // This makes sure that the autonomous stops running when
          // teleop starts running. If you want the autonomous to 
          // continue until interrupted by another command, remove
          // this line or comment it out.
          // if (autonomousCommand != null) autonomousCommand.cancel();

        // Select the control style
          controlStyle = (int) controlStyleChooser.getSelected();
          runTime = Timer.getFPGATimestamp();
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
        // updateLights();
        // callArduino();
        //Robot.i2c.write(8, 97);
        
      }
 	    
	    /************************************************************
	     *
	     * 			HELPER FUNCTIONS FOR ENTIRE ROBOT
	     * 
	     ************************************************************/           
          
          
  /**
   * Returns the status of DIO pin 23 
   * 
   * @return true if this has a CAN drivetrain
   */
  public static boolean isCanDrivetrain(){
    return !canDrivetrain.get();
  }

  /**
   * Returns the status of DIO pin 24
   *
   * @return true if this is the practice robot
   */
  public static boolean isPracticeRobot() {
    return !practiceBot.get();

  }

  /**
   * Get the name of a contron style.
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
    case 4:
      retVal = "New Gun Style";
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
      controlStyleChooser.addOption("Tank Drive", 0);
      controlStyleChooser.setDefaultOption("Gun Style Controller", 1);
      controlStyleChooser.addOption("Arcade Drive", 2);
      controlStyleChooser.addOption("GTA Drive", 3);
      controlStyleChooser.setDefaultOption("New Gun Style", 4);
    }

    /**
     * Method which determines which control (joystick) style was selected
     * returns int which is a enumeration for the control style to be implemented. The int is positive only.
     */
    public static int getControlStyleInt() {
      return (int) controlStyleChooser.getSelected();
    }

    /**
		* Adds the autos to the selector
		*/
    public void autoSelectInit() 
    {
			autoChooser = new SendableChooser<Command>();
			autoChooser.addDefault("Drive Straight", new DriveStraight(8.0));
			// autoChooser.addObject("Do Nothing", new DoNothing());
			// autoChooser.addObject("Center Auto 3 Cube", new AutoStartCenter3Cube());	        
    }
    
    /**
		* Get the name of an autonomous mode command.
		* @return the name of the auto command.
		*/
    public static String getAutoName() 
    {
      if (autonomousCommand != null) 
				return autonomousCommand.getName();
			else
				return "None";
		}

		/**
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

      if (lastGyroCalibrating && !gyroCalibrating) 
      {
        // if we've just finished calibrating the gyro, reset
        gyroDriftDetector.reset();
        curAngle = drivetrain.getHeading();
        System.out.println("Finished auto-reinit gyro");
      } 
      else if (gyroDriftDetector.update(Math.abs(curAngle - lastAngle) > (0.75 / 50.0)) && !matchStarted && !gyroCalibrating) 
      {
        // && gyroReinits < 3) {
        gyroReinits++;
        System.out.println("!!! Sensed drift, about to auto-reinit gyro (" + gyroReinits + ")");
        drivetrain.calibrateGyro();
      }

      lastAngle = curAngle;
      lastGyroCalibrating = gyroCalibrating;

    }
}
