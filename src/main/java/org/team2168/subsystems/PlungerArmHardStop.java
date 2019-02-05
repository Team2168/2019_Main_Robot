/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class PlungerArmHardStop extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DoubleSolenoid _plungerArmBrake;
  
  private static PlungerArmHardStop _instance;

  private PlungerArmHardStop()
  {
    _plungerArmBrake = new DoubleSolenoid(RobotMap.PLUNGER_ARM_BREAK_EXTENDED_PCM, 
          RobotMap.PLUNGER_ARM_BREAK_RETRACTED_PCM);
  }

  /**
	 * Singleton constructor of the plunger arm pivot
	 * 
	 */

	public static PlungerArmHardStop getInstance() {
		if (_instance == null)
			_instance = new PlungerArmHardStop();
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
