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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.PID.sensors.CanDigitalInput;
import org.team2168.commands.hatchProbePivot.DriveHatchProbePivotWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchProbePivot extends Subsystem
{
  private static TalonSRX _plungerArmPivotMotor;
  private static AveragePotentiometer _pivotPot;
  public volatile double _plungerArmPivotVoltage;
  private CanDigitalInput _pivotHallEffectSensors;
  private static HatchProbePivot _instance;

  private HatchProbePivot()
  {
    _plungerArmPivotMotor = new TalonSRX(RobotMap.PLUNGER_PIVOT_MOTOR_PDP);
    // _plungerArmPivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    // _plungerArmPivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    _pivotHallEffectSensors = new CanDigitalInput(_plungerArmPivotMotor);
    if (Robot.isPracticeRobot())
    {
      _pivotPot = new AveragePotentiometer(RobotMap.PIVOT_POSITION_POT_PBOT, RobotMap.PIVOT_POT_VOLTAGE_0_PBOT,
          RobotMap.PIVOT_POT_0_ROTATION_DEGREES_PBOT, RobotMap.PIVOT_POT_VOLTAGE_MAX_PBOT,
          RobotMap.PIVOT_POT_MAX_ROTATION_DEGREES_PBOT, RobotMap.PIVOT_AVG_ENCODER_VAL);
    }
    else
    {
      _pivotPot = new AveragePotentiometer(RobotMap.PIVOT_POSITION_POT, RobotMap.PIVOT_POT_VOLTAGE_0,
          RobotMap.PIVOT_POT_0_ROTATION_DEGREES, RobotMap.PIVOT_POT_VOLTAGE_MAX,
          RobotMap.PIVOT_POT_MAX_ROTATION_DEGREES, RobotMap.PIVOT_AVG_ENCODER_VAL);
    }

    ConsolePrinter.putNumber("HatchProbe Pivot Joystick", () -> {
      return Robot.oi.getHatchProbePivotJoystickValue();
    }, true, true);
    ConsolePrinter.putNumber("HatchProbe Pivot Motor Voltage", () -> {
      return _plungerArmPivotVoltage;
    }, true, true);
    ConsolePrinter.putNumber("HatchProbe Pivot Motor Current ", () -> {
      return Robot.pdp.getChannelCurrent(RobotMap.PLUNGER_PIVOT_MOTOR_PDP);
    }, true, true);
    ConsolePrinter.putBoolean("HatchProbe Pivot Motor", () -> {
      return !Robot.pdp.isPlungerArmPivotMotorTrip();
    }, true, false);

    ConsolePrinter.putNumber("HatchProbe Pivot Raw Pot", () -> {
      return getRawPot();
    }, true, false);
    ConsolePrinter.putNumber("HatchProbe Pivot Degrees", () -> {
      return getPotPos();
    }, true, false);

    ConsolePrinter.putBoolean("HatchProbe Pivot isForward", () -> {
      return isPivotHallEffectForward();
    }, true, false);
    ConsolePrinter.putBoolean("HatchProbe Pivot isReverse", () -> {
      return isPivotHallEffectReverse();
    }, true, false);

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
  public void drivePlungerArmPivotMotor(double speed)
  {
    if (RobotMap.PLUNGER_ARM_PIVOT_REVERSE)
      speed = -speed;
    _plungerArmPivotMotor.set(ControlMode.PercentOutput, speed);
    _plungerArmPivotVoltage = Robot.pdp.getBatteryVoltage() * speed; // not currently used
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

  public boolean isPivotHallEffectForward()
  {
    return _pivotHallEffectSensors.getForwardLimit();
  }

  public boolean isPivotHallEffectReverse()
  {
    return _pivotHallEffectSensors.getReverseLimit();
  }


  @Override
  public void initDefaultCommand()
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveHatchProbePivotWithJoystick());
  }
}
