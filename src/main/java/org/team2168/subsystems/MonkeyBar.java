
package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.commands.monkeyBar.DriveRotateMonkeyBarWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;

public class MonkeyBar extends Subsystem 
{

  private VictorSPX intakeLeft;
  private VictorSPX intakeRight;
  
  private TalonSRX rotateBarLeft;
  private TalonSRX rotateBarRight;
  

  
  private AveragePotentiometer monkeyBarRotationRight;

  private double POT_MAX_HEIGHT_LEFT;
  private double POT_MAX_HEIGHT_RIGHT;
  private double POT_MIN_HEIGHT_LEFT;
  private double POT_MIN_HEIGHT_RIGHT;

  private static MonkeyBar _instance;


  //constructors for monkey bar
  private MonkeyBar() 
  { 
    intakeLeft = new VictorSPX(RobotMap.MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP);
    intakeRight = new VictorSPX(RobotMap.MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP);

    rotateBarLeft = new TalonSRX(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);
    rotateBarRight = new TalonSRX(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);
    
    if(Robot.isPracticeRobot())
    {

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

    ConsolePrinter.putNumber("Right Monkey Bar Pot Position", () -> {return getRightPotPos();}, true, false);
    ConsolePrinter.putNumber("Right Monkey Bar Raw Pot Position", () -> {return getRightPotPosRaw();}, true, false);


    // ConsolePrinter.putNumber("Intake right motor voltage", () -> {return intakeMotorRightVoltage;}, true, false);
    // ConsolePrinter.putNumber("Intake left motor voltage", () -> {return intakeMotorLeftVoltage;}, true, false);

    // ConsolePrinter.putNumber("Rotate bar right motor voltage", () -> {return barRotateMotorRightVoltage;}, true, false);
    // ConsolePrinter.putNumber("Rotate bar left motor voltage", () -> {return barRotateMotorLeftVoltage;}, true, false);

    ConsolePrinter.putNumber("MonkeyBar Intake right motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_INTAKE_WHEELS_LEFT_PDP);}, true, false);
    ConsolePrinter.putNumber("MonkeyBar Intake left motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_INTAKE_WHEELS_RIGHT_PDP);}, true, false);

    ConsolePrinter.putNumber("MonkeyBar Pivot left motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_LEFT_PDP);}, true, false);
    ConsolePrinter.putNumber("MonkeyBar Pivot right motor current", () -> {return Robot.pdp.getChannelCurrent(RobotMap.MONKEY_BAR_ROTATE_RIGHT_PDP);}, true, false);
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
    setDefaultCommand(new DriveRotateMonkeyBarWithJoystick());
  }

  public void driveIntakeMotorLeft(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_LEFT_REVERSE)
      speed = -speed;

    intakeLeft.set(ControlMode.PercentOutput,speed);

  }

  public void driveIntakeMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_INTAKE_RIGHT_REVERSE)
      speed = -speed;

    intakeRight.set(ControlMode.PercentOutput,speed);
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

    rotateBarLeft.set(ControlMode.PercentOutput,speed);

  }

  public void driveRotateMotorRight(double speed)
  {
    if(RobotMap.MONKEY_BAR_ROTATE_RIGHT_REVERSE)
    speed = -speed;

    rotateBarRight.set(ControlMode.PercentOutput,speed);

  }
  
  public void driveRotateBarMotors(double speed) 
  {
    driveRotateMotorLeft(speed);
    driveRotateMotorRight(speed);
  }

  public boolean isLowered()
  {
    return monkeyBarRotationRight.getPos() >= POT_MIN_HEIGHT_RIGHT;

    
  }

  public boolean isStowed()
  {
    return monkeyBarRotationRight.getPos() == POT_MAX_HEIGHT_RIGHT;
  }

  public double getRightPotPosRaw()
  {
    return monkeyBarRotationRight.getRawPos();
  }

  public double getRightPotPos()
  {
    return monkeyBarRotationRight.getPos();
  }
}
