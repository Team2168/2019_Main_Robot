/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.PID.sensors.CanDigitalInput;
import org.team2168.commands.hatchProbePivot.DriveHatchProbePivotWithJoystick;
import org.team2168.commands.lift.MoveLiftToLvl1Position;
import org.team2168.commands.monkeyBarPivot.interlocks.MoveMonkeyBarToSafePositionForPivot;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchProbePivot extends Subsystem {
  public TalonSRX _plungerArmPivotMotor; // made public so can be accessed as sensor reference in another subsystem
  private static AveragePotentiometer _pivotPot;
  public volatile double _plungerArmPivotVoltage;
  private CanDigitalInput _pivotHallEffectSensors;
  private static HatchProbePivot _instance;


  public PIDPosition hatchProbePivotController;
  TCPSocketSender TCPHatchProbePivotController;

  private double _middlePosition;
  private double _safePositionMonkeyBarSide;
  private double _safePositionOtherSide;
  //cargoPos is the angle to score on the cargo ship, error is so that going to that angle will not trigger lift to go down
  private double _cargoPosition;
  
  private double _error;

  MoveMonkeyBarToSafePositionForPivot moveMonkeyBarToSafePositionForPivot;
  MoveLiftToLvl1Position moveLiftFullyDown;

  private HatchProbePivot()
  {
    _plungerArmPivotMotor = new TalonSRX(RobotMap.PLUNGER_PIVOT_MOTOR_PDP);
    //_plungerArmPivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    _plungerArmPivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    _plungerArmPivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    _pivotHallEffectSensors = new CanDigitalInput(_plungerArmPivotMotor);

    if (Robot.isPracticeRobot())
    {
      _pivotPot = new AveragePotentiometer(RobotMap.PIVOT_POSITION_POT, 
          RobotMap.PIVOT_POT_VOLTAGE_0_PBOT,
          RobotMap.PIVOT_POT_0_ROTATION_DEGREES_PBOT, 
          RobotMap.PIVOT_POT_VOLTAGE_MAX_PBOT,
          RobotMap.PIVOT_POT_MAX_ROTATION_DEGREES_PBOT, 
          RobotMap.PIVOT_AVG_ENCODER_VAL);
          _middlePosition = RobotMap.PLUNGER_ARM_MIDDLE_POS_PBOT;
          _cargoPosition = RobotMap.PLUNGER_ARM_CARGO_SHIP_POS_PBOT;
          _safePositionMonkeyBarSide = RobotMap.PLUNGER_ARM_SAFE_POS_FRONT_PBOT;
          _safePositionOtherSide = RobotMap.PLUNGER_ARM_SAFE_POS_BACK_PBOT;
          _error = RobotMap.PLUNGER_ARM_ERROR_PBOT;
    }
    else
    {
      _pivotPot = new AveragePotentiometer(RobotMap.PIVOT_POSITION_POT, 
          RobotMap.PIVOT_POT_VOLTAGE_0,
          RobotMap.PIVOT_POT_0_ROTATION_DEGREES, 
          RobotMap.PIVOT_POT_VOLTAGE_MAX,
          RobotMap.PIVOT_POT_MAX_ROTATION_DEGREES, 
          RobotMap.PIVOT_AVG_ENCODER_VAL);
          _middlePosition = RobotMap.PLUNGER_ARM_MIDDLE_POS;
          _cargoPosition = RobotMap.PLUNGER_ARM_CARGO_SHIP_POS;
          _safePositionMonkeyBarSide = RobotMap.PLUNGER_ARM_SAFE_POS_FRONT;
          _safePositionOtherSide = RobotMap.PLUNGER_ARM_SAFE_POS_BACK;
          _error = RobotMap.PLUNGER_ARM_ERROR;
    }

    hatchProbePivotController = new PIDPosition("HatchProbePivotController", 
      RobotMap.HP_PIVOT_P, 
      RobotMap.HP_PIVOT_I, 
      RobotMap.HP_PIVOT_D,
      _pivotPot, 
      RobotMap.LIFT_PID_PERIOD);

    hatchProbePivotController.setSIZE(RobotMap.LIFT_PID_ARRAY_SIZE);

    hatchProbePivotController.startThread();

    TCPHatchProbePivotController = new TCPSocketSender(RobotMap.TCP_SERVER_HP_POT_CONTROLLER, hatchProbePivotController);
    TCPHatchProbePivotController.start();
    
    ConsolePrinter.putNumber("HatchProbe Pivot Joystick", () -> {return Robot.oi.getHatchProbePivotJoystickValue();}, true, false);
    ConsolePrinter.putNumber("HatchProbe Pivot Motor Voltage", () -> {return _plungerArmPivotVoltage;}, true, false);
    ConsolePrinter.putNumber("HatchProbe Pivot Motor Current ", () -> {return Robot.pdp.getChannelCurrent(RobotMap.PLUNGER_PIVOT_MOTOR_PDP);}, true, false);
    
    //ConsolePrinter.putBoolean("HatchProbe Pivot Motor", () -> {return !Robot.pdp.isPlungerArmPivotMotorTrip();}, true, false);

    ConsolePrinter.putNumber("HatchProbe Pivot Raw Pot", () -> {return getRawPot();}, true, false);
    ConsolePrinter.putNumber("HatchProbe Pivot Degrees", () -> {return getPotPos();}, true, false);

    ConsolePrinter.putBoolean("HatchProbe Pivot isForward", () -> {return isPivotHallEffectMonkeyBar();}, true, false);
    ConsolePrinter.putBoolean("HatchProbe Pivot isReverse", () -> {return isPivotHallEffectOpposite();}, true, false);

  }

  /**
   * Singleton constructor of the plunger arm pivot
   * 
   */

  public static HatchProbePivot getInstance()
  {
    if (_instance == null)
      _instance = new HatchProbePivot();
    return _instance;
  }

  /**
   * Calls plunger arm pivot motor and creates a local variable "speed" Refers to
   * boolean in Robot map and if true, speed = - speed Uses set() command to
   * assign the new speed to plunger arm pivot motor.
   * 
   * 
   * @param double speed between -1 and 1 positive values rotate pivot to the
   *        front of the robot, negative values rotate pivot to the back of the
   *        robot, 0 is stationary
   */
  public void drivePlungerArmPivotMotorUnsafe(double speed)
  {
    if (RobotMap.PLUNGER_ARM_PIVOT_REVERSE)
      speed = -speed;
    _plungerArmPivotMotor.set(ControlMode.PercentOutput, speed);
    _plungerArmPivotVoltage = Robot.pdp.getBatteryVoltage() * speed; // not currently used

   
  }

  public void drivePlungerArmPivotMotor(double speed)
  {
    if(moveMonkeyBarToSafePositionForPivot == null)
      moveMonkeyBarToSafePositionForPivot = new MoveMonkeyBarToSafePositionForPivot();

    if(moveLiftFullyDown == null)
      moveLiftFullyDown = new MoveLiftToLvl1Position();

    // move lift fully down if not already and not already and if not on monkey bar side preparing to score on cargo ship
    if(RobotMap.PLUNGER_PIVOT_ENABLE_INTERLOCKS && (Robot.lift.isLiftFullyDown() || Robot.lift.getPotPos() <=13) && !moveLiftFullyDown.isRunning() && !isWithinCargoAngle())
    {
      moveLiftFullyDown.start();
      System.out.println("cannot Pivot, lift is not down");
    }
   // move monkey bar out of way of pivot if not already
    if(RobotMap.PLUNGER_PIVOT_ENABLE_INTERLOCKS && !Robot.monkeyBarPivot.isSafePivotPosition() && !moveMonkeyBarToSafePositionForPivot.isRunning())
    {
      moveMonkeyBarToSafePositionForPivot.start();
      System.out.println("cannot Pivot, Monkey bar is not in safe position");
    }

    // if monkey bar in safe pos, lift all the way down, or within angle needed to score on CS, drive the pivot

    if(RobotMap.PLUNGER_PIVOT_ENABLE_INTERLOCKS && Robot.monkeyBarPivot.isSafePivotPosition() && ((Robot.lift.isLiftFullyDown() || Robot.lift.getPotPos() <=13) || isWithinCargoAngle()))
    {
      if (RobotMap.PLUNGER_ARM_PIVOT_REVERSE)
        speed = -speed;
      _plungerArmPivotMotor.set(ControlMode.PercentOutput, speed);
      _plungerArmPivotVoltage = Robot.pdp.getBatteryVoltage() * speed; // not currently used
      System.out.println("pivot running");
    }
    else
    {
      if (RobotMap.PLUNGER_ARM_PIVOT_REVERSE)
        speed = -speed;
      _plungerArmPivotMotor.set(ControlMode.PercentOutput, speed);
      _plungerArmPivotVoltage = Robot.pdp.getBatteryVoltage() * speed; // not currently used
      //System.out.println("nope");
     }
  }

  /**
   * 
   * @return pot position in volts
   */
  public double getRawPot()
  {
    return _pivotPot.getRawPos();
  }

  /**
   * 
   * @return pot position in degrees from 0 to 180
   */
  public double getPotPos()
  {
    return _pivotPot.getPos();
  }

  public boolean isPivotHallEffectMonkeyBar()
  {
    return _pivotHallEffectSensors.getForwardLimit();
  }

  public boolean isPivotHallEffectOpposite()
  {
    return _pivotHallEffectSensors.getReverseLimit();
  }

  public boolean isOnMonkeyBarSide()
  {
    return getPotPos() >= 90.0;
  }

  public boolean isWithinCargoAngle()
  {
    return (getPotPos() <= _cargoPosition + _error);
  }

  public boolean isSafeToMoveLiftUp()
  {
    return (getPotPos() <= _safePositionMonkeyBarSide || getPotPos()>=_safePositionOtherSide);
  }


  @Override
  public void initDefaultCommand()
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveHatchProbePivotWithJoystick());
  }
}
