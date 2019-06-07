/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import org.team2168.commands.LEDs.AutoWithoutGamePiecePattern;
import org.team2168.commands.LEDs.HABClimbPattern;
import org.team2168.commands.LEDs.TeleopWithoutGamePiecePattern;
import org.team2168.commands.LEDs.WithGamePiecePattern;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.drivetrain.EngageDrivetrain;
import org.team2168.commands.pneumatics.StartCompressor;
import org.team2168.subsystems.CargoIntakeWheels;
import org.team2168.subsystems.CargoPunch;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.HatchFloorIntake;
import org.team2168.subsystems.HatchProbePistons;
import org.team2168.subsystems.HatchProbePivot;
import org.team2168.subsystems.HatchProbePivotBrake;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.MonkeyBarPivot;
import org.team2168.subsystems.MonkeyBarIntakeWheels;
import org.team2168.subsystems.Pneumatics;
import org.team2168.subsystems.ShifterDrivetrain;
import org.team2168.subsystems.ShifterStinger;
import org.team2168.subsystems.Stinger;
import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
public class Robot extends TimedRobot
{
  // Digital Jumper to Identify if this is practice bot or comp bot
  private static DigitalInput practiceBot;

  // Digital Jumper to Identify if this uses a CAN drivetrain
  private static DigitalInput pwmDrivetrain;

  // Operator Interface
  public static OI oi;

  // Subsystems
  public static CargoIntakeWheels cargoIntakeWheels;
  public static CargoPunch cargoPunch;
  public static Drivetrain drivetrain;
  public static ShifterStinger shifterStinger;
  public static ShifterDrivetrain shifterDrivetrain;
  public static HatchProbePivot hatchProbePivot;
  public static HatchProbePivotBrake hatchProbePivotBrake;
  public static HatchProbePistons hatchProbePistons;
  public static HatchFloorIntake hatchFloorIntake;
  public static Lift lift;
  public static MonkeyBarPivot monkeyBarPivot;
  public static MonkeyBarIntakeWheels monkeyBarIntakeWheels;
  public static Stinger stinger;
  public static Pneumatics pneumatics;
  public static LEDs leds;

  // Variables for initializing and calibrating the Gyro
  static boolean autoMode;
  private static boolean matchStarted = false;
  public static int gyroReinits;
  private double lastAngle;
  private Debouncer gyroDriftDetector = new Debouncer(1.0);
  public static boolean gyroCalibrating = false;
  private boolean lastGyroCalibrating = false;
  private double curAngle = 0.0;

  // PDP Instance
  public static PowerDistribution pdp;

  // Driverstation Instance
  public static DriverStation driverstation;

  // Driver Joystick Chooser
  static int controlStyle;
  static int throttleStyle;
  static Command autonomousCommand;
  public static SendableChooser<Command> autoChooser;
  public static SendableChooser<Number> controlStyleChooser;
  public static SendableChooser<Number> throttleVibeChooser;
  
  //boolean to keep track of climb mode
  public static boolean isClimbEnabled = false;
  public static boolean isClimbEnabledLevel2 = false;

  //Variable to track blue alliance vs red alliance
  private static boolean blueAlliance = false;

  // Keep track of time
  double runTime = Timer.getFPGATimestamp();

  //LEDs stuff
  public static WithGamePiecePattern withGamePiecePattern;
  public static AutoWithoutGamePiecePattern autoWithoutGamePiecePattern;
  public static TeleopWithoutGamePiecePattern teleopWithoutGamePiecePattern;
  public static HABClimbPattern habClimbPattern;
  private static boolean canRunGamePiecePattern = true;
  private static boolean lastHatch = false;
  private static boolean lastCargo = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit()
  {

    try
    {
      // Stop all WPILib 2018 Telementry
      LiveWindow.disableAllTelemetry();

      ConsolePrinter.init();
      ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS);

      practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);
      pwmDrivetrain = new DigitalInput(RobotMap.CAN_DRIVETRAIN_JUMPER);

      // Instantiate the subsystems
      leds = LEDs.getInstance();
      cargoIntakeWheels = CargoIntakeWheels.getInstance();
      cargoPunch = CargoPunch.getInstance();
      drivetrain = Drivetrain.getInstance();
      shifterStinger = ShifterStinger.getInstance();
      shifterDrivetrain = ShifterDrivetrain.getInstance();
      lift = Lift.getInstance();
      hatchProbePivot = HatchProbePivot.getInstance();
      hatchProbePivotBrake = HatchProbePivotBrake.getInstance();
      hatchProbePistons = HatchProbePistons.getInstance();
      hatchFloorIntake = HatchFloorIntake.getInstance();
      monkeyBarPivot = MonkeyBarPivot.getInstance();
      monkeyBarIntakeWheels = MonkeyBarIntakeWheels.getInstance();
      pneumatics = Pneumatics.getInstance();
      stinger = Stinger.getInstance();

      //init for leds and associated command
      
      withGamePiecePattern = new WithGamePiecePattern();
      autoWithoutGamePiecePattern = new AutoWithoutGamePiecePattern();
      habClimbPattern = new HABClimbPattern();
      teleopWithoutGamePiecePattern = new TeleopWithoutGamePiecePattern();

      drivetrain.calibrateGyro();
      driverstation = DriverStation.getInstance();

      // Starting PDP
      pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
      pdp.startThread();

      // Start Operator Interface
      oi = OI.getInstance();

      // enable compressor
      new StartCompressor();

      // Initialize Autonomous Selector Choices
      autoSelectInit();
      controlStyleSelectInit();
      throttleVibeSelectInit();
      
      ConsolePrinter.startThread();
      ConsolePrinter.putSendable("Control Style Chooser", () -> {return Robot.controlStyleChooser;}, true, false);
      ConsolePrinter.putSendable("Autonomous Mode Chooser", () -> {return Robot.autoChooser;}, true, false);
      ConsolePrinter.putSendable("Throttle Vibe Chooser", () -> {return Robot.throttleVibeChooser;}, true, false);
      ConsolePrinter.putString("AutoName", () -> {return Robot.getAutoName();}, true, false);
      ConsolePrinter.putString("Control Style Name", () -> {return Robot.getControlStyleName();}, true, false);
      ConsolePrinter.putNumber("gameClock", () -> {return driverstation.getMatchTime();}, true, false);
      ConsolePrinter.putNumber("Robot Pressure", () -> {return Robot.pneumatics.getPSI();}, true, false);
      ConsolePrinter.putBoolean("Is Practice Bot", () -> {return isPracticeRobot();}, true, false);
      ConsolePrinter.putBoolean("Is Climb Mode", () -> {return isClimbMode();}, true, false);
      
      
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

      // Show Stack Trace on Driverstration like before
      DriverStation.reportError("Unhandled exception instantiating robot" + throwable.toString(),
          throwable.getStackTrace());
      DriverStation.reportWarning("Robots should not quit, but yours did!", false);
      DriverStation.reportError("Could not instantiate robot!", false);
      System.exit(1);
      return;
    }

  }

  /************************************************************
   *
   * Robot State Machine
   * 
   ************************************************************/

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
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

    // If we are not in a match allow Gyro to be recalibrated in Disabled even if a
    // previous
    // calibration was performed, we disable this in a match so that if we ever die
    // in a match,
    // we don't try to recalibrate a moving robot.
    if (driverstation.isFMSAttached())
      drivetrain.startGyroCalibrating();

    drivetrain.calibrateGyro();
    
    teleopWithoutGamePiecePattern.cancel();
    // disabled pattern has to be triggered directly 
    if (Robot.onBlueAlliance())
    {
      Robot.leds.writePatternOneColor(RobotMap.PATTERN_2168, 160, 255, 255);
    }
    else
    {
      Robot.leds.writePatternOneColor(RobotMap.PATTERN_2168, 0, 255, 255);
    }

    drivetrain.limelightPosController.Pause();
  }

  public void disabledPeriodic()
  {

    // Keep track of Gunstyle Controller Variables

    // callArduino();
    getControlStyleInt();
    controlStyle = (int) controlStyleChooser.getSelected();
    throttleStyle = (int) throttleVibeChooser.getSelected();
    autonomousCommand = (Command) autoChooser.getSelected();

    Scheduler.getInstance().run();
    Drivetrain.getInstance().limelight.setPipeline(8);

    // Check to see if the gyro is drifting, if it is re-initialize it.
    gyroReinit();
    teleopWithoutGamePiecePattern.cancel();
  }

  public void autonomousInit()
  {
    autoMode = true;

    matchStarted = true;
    drivetrain.stopGyroCalibrating();
    drivetrain.resetGyro();

    autonomousCommand = (Command) autoChooser.getSelected();

    // schedule the autonomous command
    if (autonomousCommand != null)
      autonomousCommand.start();

    Scheduler.getInstance().add(new EngageDrivetrain());
  }

  /**
   * This function is called periodically during autonomous
   */
  public void autonomousPeriodic()
  {
    autoMode = true;
    Scheduler.getInstance().run();
  }

  /**
   * This function called prior to robot entering Teleop Mode
   */
  public void teleopInit()
  {
    // callArduino();
    autoMode = false;
    matchStarted = true;
    drivetrain.stopGyroCalibrating();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    Scheduler.getInstance().add(new EngageDrivetrain());

    if (autonomousCommand != null) autonomousCommand.cancel();

    // Select the control style
    controlStyle = (int) controlStyleChooser.getSelected();
    runTime = Timer.getFPGATimestamp();
  }

  /**
   * This function is called periodically during operator control
   */
  public void teleopPeriodic()
  {
    if((int) throttleVibeChooser.getSelected() == 0) 
    {
      Robot.oi.driverJoystick.setRumble(RumbleType.kLeftRumble, Math.abs(Robot.oi.getGunStyleYValue()));
    }

    SmartDashboard.putNumber("TeleopLoopTime", Timer.getFPGATimestamp() - runTime);
    runTime = Timer.getFPGATimestamp();

    autoMode = false;
    Scheduler.getInstance().run();

    controlStyle = (int) controlStyleChooser.getSelected();
    throttleStyle = (int) throttleVibeChooser.getSelected();
    // if(hatchProbePistons.isHatchPresentLimitSwitch() && canRunGamePiecePattern)
    // {
    //   withGamePiecePattern.start();
    //   canRunGamePiecePattern = false;
    //   lastHatch = true;
    // }
    // else if(cargoIntakeWheels.isCargoPresent()  && canRunGamePiecePattern)
    // {
    //   withGamePiecePattern.start();
    //   canRunGamePiecePattern = false;
    //   lastCargo = true;
    // }
    // if(lastHatch && !hatchProbePistons.isHatchPresentLimitSwitch() && !withGamePiecePattern.isRunning())
    // {
    //   canRunGamePiecePattern = true;
    //   lastHatch = false;
    // }
    // else if(lastCargo && !cargoIntakeWheels.isCargoPresent() && !withGamePiecePattern.isRunning())
    // {
    //   canRunGamePiecePattern = true;
    //   lastCargo = false;
    // }
    
  }

  /************************************************************
   *
   * HELPER FUNCTIONS FOR ENTIRE ROBOT
   * 
   ************************************************************/

  /**
   * Returns the status of DIO pin 23
   * 
   * @return true if this has a CAN drivetrain
   */
  public static boolean isPWMDrivetrain()
  {
    return !pwmDrivetrain.get();
  }

  /**
   * Returns the status of DIO pin 24
   *
   * @return true if this is the practice robot
   */
  public static boolean isPracticeRobot()
  {
    return !practiceBot.get();

  }

  /**
   * Get the name of a contron style.
   * 
   * @return the name of the control style.
   */
  public static String getControlStyleName()
  {
    String retVal = "";

    switch (controlStyle)
    {
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
  public void controlStyleSelectInit()
  {
    controlStyleChooser = new SendableChooser<>();
    controlStyleChooser.addOption("Tank Drive", 0);
    controlStyleChooser.setDefaultOption("Gun Style Controller", 1);
    controlStyleChooser.addOption("Arcade Drive", 2);
    controlStyleChooser.addOption("GTA Drive", 3);
    controlStyleChooser.setDefaultOption("New Gun Style", 4);
  }

  /**
   * Method which determines which control (joystick) style was selected returns
   * int which is a enumeration for the control style to be implemented. The int
   * is positive only.
   */
  public static int getControlStyleInt()
  {
    return (int) controlStyleChooser.getSelected();
  }

  public static int getTrottleVibeInt()
  {
    return (int) throttleVibeChooser.getSelected();
  }

  public void throttleVibeSelectInit()
  {
    throttleVibeChooser = new SendableChooser<>();
    throttleVibeChooser.addOption("Throttle Vibe ON", 0);
    throttleVibeChooser.setDefaultOption("Throttle Vibe OFF", 1);
  }

  /**
   * Adds the autos to the selector
   */
  public void autoSelectInit()
  {
    autoChooser = new SendableChooser<Command>();
    // autoChooser.addDefault("Drive Straight", new DriveStraight(8.0));
    autoChooser.addObject("Do Nothing", new DoNothing());
    // autoChooser.addObject("Center Auto 3 Cube", new AutoStartCenter3Cube());
  }

  /**
   * Get the name of an autonomous mode command.
   * 
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
  public static boolean isAutoMode()
  {
    return autoMode;

  }

    /**
   * @return true if the robot is in climb mode
   */
  public static boolean isClimbMode()
  {
    return isClimbEnabled;

  }

  public static boolean onBlueAlliance() {
		return driverstation.getAlliance() == DriverStation.Alliance.Blue;

  }

  // public static void setCanRunGamePiecePattern(boolean input)
  // {
  //   canRunGamePiecePattern = input;
  // }

  // public static boolean returnCanRunGamePiecePattern()
  // {
  //   return canRunGamePiecePattern;
  // }
	

  /**
   * Method which checks to see if gyro drifts and resets the gyro. Call this in a
   * loop.
   */
  private void gyroReinit()
  {
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
    else if (gyroDriftDetector.update(Math.abs(curAngle - lastAngle) > (0.75 / 50.0)) && !matchStarted
        && !gyroCalibrating)
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
