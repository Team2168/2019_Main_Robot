/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.Subsystems;


import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team2168.PID.sensors.AveragePotentiometer;
/**
 * Add your docs here.
 */



public class MonkeyBar extends Subsystem 
{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static VictorSP MotorIntake1;
  private static VictorSP MotorIntake2;
  
  private static VictorSP RotateBar1;
  private static VictorSP RotateBar2;

  
  private static AveragePotentiometer monkeyBarRotationCheck1;
  private static AveragePotentiometer monkeyBarRotationCheck2;

  //constructors for monkey bar
  public MonkeyBar() 
  { 
    MotorIntake1 = new VictorSP(RobotMap.INTAKE_ARM_MOTOR_RIGHT);
    MotorIntake2 = new VictorSP(RobotMap.INTAKE_ARM_MOTOR_LEFT);

    RotateBar1 = new VictorSP(RobotMap.ROTATE_ARM_MOTOR_RIGHT);
    RotateBar2 = new VictorSP(RobotMap.ROTATE_ARM_MOTOR_LEFT);
    
    monkeyBarRotationCheck1 = new AveragePotentiometer
    (RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER1, RobotMap.MONKEY_BAR_POT_VOLTAGE_0, 
    RobotMap.MONKEY_BAR_0_ANGLE_DEGREES, RobotMap.MONKEY_BAR_POT_VOLTAGE_MAX, 
    RobotMap.MONKEY_BAR_LIFT_POT_MAX_ROTATION, RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

    monkeyBarRotationCheck2 = new AveragePotentiometer
    (RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER2, RobotMap.MONKEY_BAR_POT_VOLTAGE_0, 
    RobotMap.MONKEY_BAR_0_ANGLE_DEGREES, RobotMap.MONKEY_BAR_POT_VOLTAGE_MAX, 
    RobotMap.MONKEY_BAR_LIFT_POT_MAX_ROTATION, RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);
  }

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * This program will rotate the monkey arm up and down
   *  
   */  
  public void driveIntakeMotor1(double speed)
  {
    MotorIntake1.set(speed);
  }

  public void driveIntakeMotor2(double speed)
  {
    MotorIntake2.set(speed);
  }

  public void driveIntakeAll(double speed) 
  {
    driveIntakeMotor1(speed);
    driveIntakeMotor2(-speed);
  }
  
  public boolean checkFullPosition()
  {
    monkeyBarRotationCheck1.getPos();
    monkeyBarRotationCheck2.getPos();

    boolean checkPositionUp1 = monkeyBarRotationCheck1.getPos() == 
    RobotMap.MONKEY_BAR_POT_VOLTAGE_MAX;
    
    boolean checkPositionUp2 = monkeyBarRotationCheck2.getPos() == 
    RobotMap.MONKEY_BAR_POT_VOLTAGE_MAX;

    return checkPositionUp1 && checkPositionUp2;
  }

  public void driveBarRotate1(double speed)
  {
    RotateBar1.set(speed);
  }

  public void driveBarRotate2(double speed)
  {
    RotateBar2.set(speed);
  }
  
  public void driveRotateBarMotors(double speed) 
  {
    driveBarRotate1(speed);
    driveBarRotate2(-speed);
  }

  public boolean checkZeroPosition()
  {
    monkeyBarRotationCheck1.getPos();
    monkeyBarRotationCheck2.getPos();

    boolean checkPositionDown1 = monkeyBarRotationCheck1.getPos() == 
    RobotMap.MONKEY_BAR_POT_VOLTAGE_0;
    
    boolean checkPositionDown2 = monkeyBarRotationCheck2.getPos() == 
    RobotMap.MONKEY_BAR_POT_VOLTAGE_0;

    return checkPositionDown1 && checkPositionDown2;
  }

  public double checkCurrentPosition1()
  {
    return monkeyBarRotationCheck1.getPos();
  }

  public double checkCurrentPosition2()
  {
    return monkeyBarRotationCheck2.getPos();
  }
}
