
package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class MonkeyBar extends Subsystem 
{

  private VictorSP intakeLeft;
  private VictorSP intakeRight;
  
  private VictorSP rotateBarLeft;
  private VictorSP rotateBarRight;

  
  private AveragePotentiometer monkeyBarRotationLeft;
  private AveragePotentiometer monkeyBarRotationRight;

  private double POT_MAX_HEIGHT_LEFT;
  private double POT_MAX_HEIGHT_RIGHT;
  private double POT_MIN_HEIGHT_LEFT;
  private double POT_MIN_HEIGHT_RIGHT;

  private static MonkeyBar _instance;


  //constructors for monkey bar
  private MonkeyBar() 
  { 
    intakeLeft = new VictorSP(RobotMap.MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP);
    intakeRight = new VictorSP(RobotMap.MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP);

    rotateBarLeft = new VictorSP(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);
    rotateBarRight = new VictorSP(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);
    
    if(Robot.isPracticeRobot())
    {
      monkeyBarRotationLeft = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_LEFT, 
        RobotMap.MONKEY_BAR_LEFT_POT_VOLTAGE_0_PBOT, 
        RobotMap.MONKEY_BAR_LEFT_ANGLE_DEGREES_0_PBOT, 
        RobotMap.MONKEY_BAR_LEFT_POT_VOLTAGE_MAX_PBOT, 
        RobotMap.MONKEY_BAR_LEFT_POT_MAX_ROTATION_PBOT, 
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

      monkeyBarRotationRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT, 
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0_PBOT, 
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT, 
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX_PBOT, 
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT, 
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

        POT_MAX_HEIGHT_LEFT = RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT;
        POT_MAX_HEIGHT_RIGHT = RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION_PBOT;
        POT_MIN_HEIGHT_LEFT = RobotMap.MONKEY_BAR_LEFT_ANGLE_DEGREES_0_PBOT;
        POT_MIN_HEIGHT_RIGHT = RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0_PBOT;
    }
    else
    {
      monkeyBarRotationLeft = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_LEFT, 
        RobotMap.MONKEY_BAR_LEFT_POT_VOLTAGE_0, 
        RobotMap.MONKEY_BAR_LEFT_ANGLE_DEGREES_0, 
        RobotMap.MONKEY_BAR_LEFT_POT_VOLTAGE_MAX, 
        RobotMap.MONKEY_BAR_LEFT_POT_MAX_ROTATION, 
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

      monkeyBarRotationRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT, 
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_0, 
        RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0, 
        RobotMap.MONKEY_BAR_RIGHT_POT_VOLTAGE_MAX, 
        RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION, 
        RobotMap.MONKEY_BAR_AVG_ENCODER_VAL);

      POT_MAX_HEIGHT_LEFT = RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION;
      POT_MAX_HEIGHT_RIGHT = RobotMap.MONKEY_BAR_RIGHT_POT_MAX_ROTATION;
      POT_MIN_HEIGHT_LEFT = RobotMap.MONKEY_BAR_LEFT_ANGLE_DEGREES_0;
      POT_MIN_HEIGHT_RIGHT = RobotMap.MONKEY_BAR_RIGHT_ANGLE_DEGREES_0;
    }
  }

  /**
  * Singleton constructor of the plunger arm pivot
  * 
  */
  public static MonkeyBar getInstance() {
   if (_instance == null)
     _instance = new MonkeyBar();
   return _instance;
 }

  @Override
  public void initDefaultCommand() 
  {

  }

  public void driveIntakeMotorLeft(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_LEFT_REVERSE)
      speed = -speed;

    intakeLeft.set(speed);

  }

  public void driveIntakeMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_RIGHT_REVERSE)
      speed = -speed;

    intakeRight.set(speed);
  }

  public void driveIntakeAll(double speed) 
  {
    driveIntakeMotorLeft(speed);
    driveIntakeMotorRight(speed);
  }
  
  public void driveRotateMotorLeft(double speed)
  {
    if(RobotMap.MONKEY_BAR_ROTATE_LEFT_REVERSE)
      speed = -speed;

    rotateBarLeft.set(speed);

  }

  public void driveRotateMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_ROTATE_RIGHT_REVERSE)
    speed = -speed;

    rotateBarRight.set(speed);

  }
  
  public void driveRotateBarMotors(double speed) 
  {
    driveRotateMotorLeft(speed);
    driveRotateMotorRight(speed);
  }

  public boolean isLowered()
  {
    boolean checkPositionDown1 = monkeyBarRotationLeft.getPos() >= POT_MIN_HEIGHT_LEFT;
    boolean checkPositionDown2 = monkeyBarRotationRight.getPos() >= POT_MIN_HEIGHT_RIGHT;

    return checkPositionDown1 && checkPositionDown2;
  }

  public boolean isStowed()
  {
    boolean checkPositionUp1 = monkeyBarRotationLeft.getPos() == POT_MAX_HEIGHT_LEFT;
    boolean checkPositionUp2 = monkeyBarRotationRight.getPos() == POT_MAX_HEIGHT_RIGHT;

    return checkPositionUp1 && checkPositionUp2;
  }

  public double getLeftPotPosRaw()
  {
    return monkeyBarRotationLeft.getRawPos();
  }

  public double getRightPotPosRaw()
  {
    return monkeyBarRotationLeft.getRawPos();
  }

  public double getLeftPotPos()
  {
    return monkeyBarRotationLeft.getPos();
  }

  public double getRightPotPos()
  {
    return monkeyBarRotationRight.getPos();
  }
}
