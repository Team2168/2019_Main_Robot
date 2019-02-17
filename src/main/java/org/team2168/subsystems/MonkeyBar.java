
package org.team2168.Subsystems;


import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team2168.PID.sensors.AveragePotentiometer;




public class MonkeyBar extends Subsystem 
{

  private static VictorSP MotorIntake1;
  private static VictorSP MotorIntake2;
  
  private static VictorSP RotateBar1;
  private static VictorSP RotateBar2;

  
  private static AveragePotentiometer monkeyBarRotation1;
  private static AveragePotentiometer monkeyBarRotation2;

  private final boolean _isReversed1;
  private final boolean _isReversed2;
  
  //constructors for monkey bar
  public MonkeyBar() 
  { 
    MotorIntake1 = new VictorSP(RobotMap.INTAKE_ARM_MOTOR_RIGHT);
    MotorIntake2 = new VictorSP(RobotMap.INTAKE_ARM_MOTOR_LEFT);

    RotateBar1 = new VictorSP(RobotMap.ROTATE_ARM_MOTOR_RIGHT);
    RotateBar2 = new VictorSP(RobotMap.ROTATE_ARM_MOTOR_LEFT);
    
    monkeyBarRotation1 = new AveragePotentiometer
    (RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER1, RobotMap.MONKEY_BAR_1_POT_VOLTAGE_0, 
    RobotMap.MONKEY_BAR_1_ANGLE_DEGREES_0, RobotMap.MONKEY_BAR_1_POT_VOLTAGE_MAX, 
    RobotMap.MONKEY_BAR_1_POT_MAX_ROTATION, RobotMap.MONKEY_BAR_1_AVG_ENCODER_VAL);

    monkeyBarRotation2 = new AveragePotentiometer
    (RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER2, -RobotMap.MONKEY_BAR_2_POT_VOLTAGE_0, 
    -RobotMap.MONKEY_BAR_2_ANGLE_DEGREES_0, -RobotMap.MONKEY_BAR_2_POT_VOLTAGE_MAX, 
    -RobotMap.MONKEY_BAR_2_POT_MAX_ROTATION, -RobotMap.MONKEY_BAR_2_AVG_ENCODER_VAL);
  }

  @Override
  public void initDefaultCommand() 
  {

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
  
  public boolean isStowed()
  {
    monkeyBarRotation1.getPos();
    monkeyBarRotation2.getPos();

    boolean checkPositionUp1 = monkeyBarRotation1.getPos() <= 
    RobotMap.MONKEY_BAR_1_POT_VOLTAGE_MAX;
    
    boolean checkPositionUp2 = monkeyBarRotation2.getPos() <= 
    RobotMap.MONKEY_BAR_2_POT_VOLTAGE_MAX;

    return checkPositionUp1 && checkPositionUp2;
  }

  public void driveBarRotate1(double speed)
  {
    if (_isReversed1)
    {
      RotateBar1.set(-speed);
    }
    else 
    {
      RotateBar1.set(speed);
    }
  }

  public void driveBarRotate2(double speed)
  {
    if (isReversed2)
    {
      RotateBar2.set(-speed);
    }
    else
    {
      RotateBar2.set(speed);
    }
  }
  
  public void driveRotateBarMotors(double speed) 
  {
    driveBarRotate1(speed);
    driveBarRotate2(speed);
  }

  public boolean isLowered()
  {
    monkeyBarRotation1.getPos();
    monkeyBarRotation2.getPos();

    boolean checkPositionDown1 = monkeyBarRotation1.getPos() >= 
    RobotMap.MONKEY_BAR_1_POT_VOLTAGE_0;
    
    boolean checkPositionDown2 = monkeyBarRotation2.getPos() >= 
    RobotMap.MONKEY_BAR_2_POT_VOLTAGE_0;

    return checkPositionDown1 && checkPositionDown2;
  }

  public double checkCurrentPosition1()
  {
    return monkeyBarRotation1.getPos();
  }

  public double checkCurrentPosition2()
  {
    return monkeyBarRotation2.getPos();
  }
}
