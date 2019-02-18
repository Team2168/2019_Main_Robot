/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class PlungerArmPivot extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //hardware
  private static VictorSP _plungerArmPivotMotor; 
  private static AveragePotentiometer _pivotPot;

   public volatile double _plungerArmPivotVoltage; //not currently used
  private static PlungerArmPivot _instance;


  private PlungerArmPivot()
  {
    _plungerArmPivotMotor = new VictorSP(RobotMap.PLUNGER_ARM_PIVOT_MOTOR);
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

    ConsolePrinter.putNumber("Plunger Arm Pivot", () -> {return Robot.oi.getDrivePlungerArmPivotJoystickValue();}, true, true);
		ConsolePrinter.putNumber("Plunger arm Pivot Motor Voltage", () -> {return _plungerArmPivotVoltage;}, true, true);
		ConsolePrinter.putNumber("Plunger Arm Pivot Motor Current ", () -> {
			return Robot.pdp.getChannelCurrent(RobotMap.PLUNGER_ARM_PIVOT_MOTOR_PDP);
    }, true, true);
    ConsolePrinter.putBoolean("Plunger Arm Pivot Motor", () -> {return !Robot.pdp.isPlungerArmPivotMotorTrip();}, true, false);

		ConsolePrinter.putNumber("Plunger Arm Pivot Raw Pot", () -> {return getRawPot();}, true, false);
		ConsolePrinter.putNumber("Plunger Arm Pivot Degrees", () -> {return getPotPos();}, true, false);

  
  }

  /**
   * Singleton constructor of the plunger arm pivot
   * 
   */

  public static PlungerArmPivot getInstance() {
    if (_instance == null)
      _instance = new PlungerArmPivot();
    return _instance;
  }

  /**
   * Calls plunger arm pivot motor and creates a local variable "speed" Refers to boolean in
   * Robot map and if true, speed = - speed Uses set() command to assign the new
   * speed to plunger arm pivot motor. 
   * 
   * 
   * @param double speed between -1 and 1 
   *      positive values rotate pivot to the front of the robot,
   *      negative values rotate pivot to the back of the robot,
   *      0 is stationary
   */
  public void drivePlungerArmPivotMotor(double speed)
  { 
    if (RobotMap.PLUNGER_ARM_PIVOT_REVERSE)
      speed = -speed;
    _plungerArmPivotMotor.set(speed);
    _plungerArmPivotVoltage = Robot.pdp.getBatteryVoltage() * speed; //not currently used
  }

  /**
   * 
   * @return pot position in volts
   */
  public double getRawPot() {
    return _pivotPot.getRawPos();
  }

  /**
   * 
   * @return pot position in degrees from 0 to 180
   */
  public double getPotPos() {
    return _pivotPot.getPos();
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
