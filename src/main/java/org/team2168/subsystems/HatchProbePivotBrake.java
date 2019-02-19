/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


  public class HatchProbePivotBrake extends Subsystem {

  private static DoubleSolenoid _plungerArmBrake;
  private static HatchProbePivotBrake _instance;

  private HatchProbePivotBrake()
  {
    _plungerArmBrake = new DoubleSolenoid(RobotMap.PCM_CAN_ID_LIFT,RobotMap.PROBE_ROTATE_BRAKE_EXTENDED_PCM, RobotMap.PROBE_ROTATE_BRAKE_RETRACTED_PCM);

    ConsolePrinter.putBoolean("HatchProbePivotBrakeEngaged", () -> {return Robot.hatchProbePivotBrake.isEngaged();}, true, false);
  }

  /**
   * Singleton constructor of the plunger arm pivot
   * 
   */

  public static HatchProbePivotBrake getInstance() {
    if (_instance == null)
      _instance = new HatchProbePivotBrake();
    return _instance;
  }

  /**
   * engages brake to stop the plunger arm 
   */
  public void engage()
  {
    _plungerArmBrake.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * disengages brake to allow the plunger arm to move again
   */
  public void disengage()
  {
    _plungerArmBrake.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * @return true if last commanded shift was to engagePlungerArmBrake
   */
  public boolean isEngaged()
  {
    return _plungerArmBrake.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean isDisengaged()
  {
    return _plungerArmBrake.get() == DoubleSolenoid.Value.kReverse; 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  }
