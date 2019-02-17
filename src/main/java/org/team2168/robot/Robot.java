/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.robot;


import org.team2168.Subsystems.MonkeyBar;
import org.team2168.subsystem.CargoIntake;
import org.team2168.subsystems.FloorHatchMechanism;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.DrivetrainStingerShifter;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.LiftHardStop;
import org.team2168.subsystems.PlungerArmHardStop;
import org.team2168.subsystems.PlungerArmPivot;
import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team2168.subsystems.Stinger;
import org.team2168.subsystems.HatchPlunger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */  
public class Robot extends TimedRobot {
  
  private static final String kDefaultAuto = "Default";
  
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();





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
  public static LiftHardStop liftHardStop;
  public static PlungerArmPivot plungerArmPivot;
  public static PlungerArmHardStop plungerArmHardStop;
  public static HatchPlunger hatchPlunger;
  public static FloorHatchMechanism floorHatchMechanism;
  public static Stinger stinger;
  public static HatchPlunger hatchPlunger;
  public static FloorHatchMechanism floorHatchMechanism;
  
  //THIS IS INCORRECT
  public static MonkeyBar monkeybar = new MonkeyBar();

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
  public static SendableChooser<Number> controlStyleChooser;


  double runTime = Timer.getFPGATimestamp();

  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = OI.getInstance();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    cargointake = new CargoIntake();
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    hatchPlunger = new HatchPlunger();
    floorHatchMechanism = FloorHatchMechanism.getInstance();

    practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);
    canDrivetrain = new DigitalInput(RobotMap.CAN_DRIVETRAIN_JUMPER);

    //Instantiate the subsystems
    drivetrain = Drivetrain.getInstance();
    drivetrainStingerShifter = DrivetrainStingerShifter.getInstance();
    lift = Lift.getInstance();
    liftHardStop = LiftHardStop.getInstance();
    plungerArmPivot = PlungerArmPivot.getInstance();
    plungerArmHardStop = PlungerArmHardStop.getInstance();

    pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
    pdp.startThread();

    //Start Operator Interface
    oi = OI.getInstance();
    
    //Initialize Control Selector Choices
    controlStyleSelectInit();
    
    System.out.println("Robot Initialization Complete!!");
  }

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
        // updateLights();
        // callArduino();
        //Robot.i2c.write(8, 97);
        
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
   * Returns the status of DIO pin 25 
   * 
   * @return true if this has a CAN drivetrain
   */
  public static boolean isCanDrivetrain(){
    return !canDrivetrain.get();
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

    public static int getControlStyleInt() {
      return (int) controlStyleChooser.getSelected();
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
}
