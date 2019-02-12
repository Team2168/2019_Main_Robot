/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.robot;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.DrivetrainStingerShifter;
import org.team2168.subsystems.Lift;
import org.team2168.subsystems.LiftHardStop;
import org.team2168.subsystems.PlungerArmHardStop;
import org.team2168.subsystems.PlungerArmPivot;
import org.team2168.utils.PowerDistribution;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.TimedRobot;
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
  public static Drivetrain drivetrain;
  public static DrivetrainStingerShifter drivetrainStingerShifter;
  public static Lift lift;
  public static LiftHardStop liftHardStop;
  public static PlungerArmPivot plungerArmPivot;
  public static PlungerArmHardStop plungerArmHardStop;

  //PDP Instance
  public static PowerDistribution pdp;

  //Driver Joystick Chooser
  static int controlStyle;
  public static SendableChooser<Number> controlStyleChooser;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    practiceBot = new DigitalInput(RobotMap.PRACTICE_BOT_JUMPER);
    canDrivetrain = new DigitalInput(RobotMap.CAN_DRIVETRAIN_JUMPER);

    //Instantiate the subsystems
    drivetrain = Drivetrain.getInstance();
    drivetrainStingerShifter = DrivetrainStingerShifter.getInstance();
    lift = Lift.getInstance();
    liftHardStop = LiftHardStopgetInstance();
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

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
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
}
