/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HatchProbeWheels extends Subsystem {
  private TalonSRX _intakeMotor;
  private static HatchProbeWheels _instance;

  private HatchProbeWheels() {
    _intakeMotor = new TalonSRX(RobotMap.HATCH_INTAKE_MOTOR_PDP);
  }

  public static HatchProbeWheels getInstance() {
    if(_instance == null){
      _instance = new HatchProbeWheels();
    }
    return _instance;
  }

  public void drive(double speed) {
    if(RobotMap.HATCH_INTAKE_MOTOR_REVERSE) {
      speed = -speed;
    }

    _intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
