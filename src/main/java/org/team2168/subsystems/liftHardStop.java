/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.commands.extendLiftBreak;
import org.team2168.commands.retractLiftBreak;
import org.team2168.Robot;
import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */


public class LiftHardStop extends Subsystem {

    /**
   * a double solenoid with a rubber pad on the end to stop a gear in the lift gearbox to "lock" the lift at the specified position
   */
  public static DoubleSolenoid liftBreak;



  
  public LiftHardStop(){
    liftBreak=new DoubleSolenoid(RobotMap.LIFT_BRAKE_ENGAGE_PCM, RobotMap.LIFT_BRAKE_DISENGAGE_PCM);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static boolean getSolenoidPosition(){
    boolean _solenoidPos;
    if(liftBreak.get()==Value.kForward){
      _solenoidPos=true;
    }
    else{
      _solenoidPos=false;
    }
    return _solenoidPos;
  }
  /**
   * this method sets the value of the break to forward to extend the solenoid
   */
  public static void extendSolenoid(){
    liftBreak.set(Value.kForward);
  }
  /**
   * this method sets the value of the break to reverse to contract the solenoid
   */
  public static void contractSolenoid(){
    liftBreak.set(Value.kReverse);
  }




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new extendLiftBreak());
    setDefaultCommand(new retractLiftBreak());
  }
}
/**
  private static void baseExtendSolenoid(){
  if(getSolenoidPosition()==false){
    liftBreak.set(Value.kForward);
  }
}
private static void baseContractSolenoid(){
  if(getSolenoidPosition()==true){
    liftBreak.set(Value.kReverse);
public static void extendSolenoid(){
  if (Robot.lift.liftMotor1.get()==0.0 && Lift.liftMotor2.get()==0.0){
    baseExtendSolenoid();
  }
}

public static void contractSolenoid() {
  if (Robot.lift.liftMotor1.get()>=0.0 && Lift.liftMotor2.get()>=0.0){
    baseContractSolenoid();
*/

