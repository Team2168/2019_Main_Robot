/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.trajectory.OneDimensionalMotionProfilingNoConstraint;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveStingerPIDPath2 extends Command {

  private double[] pos;
  private double[] vel;
  private double[] accel;
  

  private double setPoint;
  OneDimensionalMotionProfilingNoConstraint motion;
	int counter;
  double ff_term = 1.1;//1.6
  double ff_term_accel = 0.075;

  private double maxSpeed;
  private double minSpeed;
  private double error = 2; //Rotational degree error, 0 never ends

  private double time;
  private double start;
  private double end;



  private DriveStingerPIDPath2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    this.setPoint = Robot.drivetrain.leftStingerController.getSetPoint();
    this.maxSpeed = 1.0;
    this.minSpeed = 0.0;

    SmartDashboard.putNumber("FF_term_sting", ff_term);
    //SmartDashboard.putNumber("FF_term_accel_sting", 0);
    
  }

  public DriveStingerPIDPath2(double end, double time)
  {
    this();
    this.start = 0;
    this.end = end;
    this.time = time;
  }

  public DriveStingerPIDPath2(double end, double time, boolean absolute)
  {
    this();
    if(absolute)
      this.start = Robot.monkeyBarPivot.getRightPotPos();
    else
   
    this.start = 0;
    this.end = end;
    this.time = time;
  }

  public DriveStingerPIDPath2(double start, double end, double time)
  {
    this();
    this.start = start;
    this.end = end;
    this.time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {

    motion = new OneDimensionalMotionProfilingNoConstraint(start,end,time); 
    
    this.pos = motion.pos;
    this.vel = motion.vel;
    this.accel = motion.acc;

    Robot.drivetrain.resetLeftStingerPosition();
    Robot.drivetrain.resetRightStingerPosition();
    Robot.drivetrain.leftStingerController.reset();
    // Robot.drivetrain.leftStingerController.setpGain(RobotMap.STINGER_AUTO_LEFT_POSITION_P);
    // Robot.drivetrain.leftStingerController.setiGain(RobotMap.STINGER_AUTO_LEFT_POSITION_I);
    // Robot.drivetrain.leftStingerController.setdGain(RobotMap.STINGER_AUTO_LEFT_POSITION_D);
    Robot.drivetrain.leftStingerController.setSetPoint(this.pos);
    Robot.drivetrain.leftStingerController.setMaxPosOutput(maxSpeed);
    Robot.drivetrain.leftStingerController.setMaxNegOutput(-maxSpeed);
    Robot.drivetrain.leftStingerController.setMinPosOutput(minSpeed);
    Robot.drivetrain.leftStingerController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.leftStingerController.setAcceptErrorDiff(error);
    Robot.drivetrain.leftStingerController.Enable();

    Robot.drivetrain.rightStingerController.reset();
    // Robot.drivetrain.rightStingerController.setpGain(RobotMap.STINGER_AUTO_RIGHT_POSITION_P);
    // Robot.drivetrain.rightStingerController.setiGain(RobotMap.STINGER_AUTO_RIGHT_POSITION_I);
    // Robot.drivetrain.rightStingerController.setdGain(RobotMap.STINGER_AUTO_RIGHT_POSITION_D);
    Robot.drivetrain.rightStingerController.setSetPoint(this.pos);
    Robot.drivetrain.rightStingerController.setMaxPosOutput(maxSpeed);
    Robot.drivetrain.rightStingerController.setMaxNegOutput(-maxSpeed);
    Robot.drivetrain.rightStingerController.setMinPosOutput(minSpeed);
    Robot.drivetrain.rightStingerController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.rightStingerController.setAcceptErrorDiff(error);
    Robot.drivetrain.rightStingerController.Enable();

    counter = 0;

    System.out.print("Lenght: "+ pos.length);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    ff_term = SmartDashboard.getNumber("FF_term_sting", ff_term);
   // double ff_term_accel = SmartDashboard.getNumber("FF_term_accel_sting", 0);

    if (counter < pos.length)
    {
      double pidLSpeed = Robot.drivetrain.leftStingerController.getControlOutput();
      double pidRSpeed = Robot.drivetrain.rightStingerController.getControlOutput();
      double ff_Speed = (ff_term  * vel[counter]) / (Robot.pdp.getBatteryVoltage());
      double ff_accel = (ff_term_accel  * accel[counter]) / (Robot.pdp.getBatteryVoltage());
      Robot.drivetrain.tankDrive(ff_Speed+pidLSpeed+ff_accel,ff_Speed+pidRSpeed+ff_accel);
      //Robot.drivetrain.tankDrive(ff_Speed+ff_accel,ff_Speed+ff_accel);
      //System.out.println(ff_Speed+pidLSpeed);
    }
    else
      Robot.drivetrain.tankDrive(0.0,0.0);


      counter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (counter >= pos.length);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.leftStingerController.Pause();
    Robot.drivetrain.rightStingerController.Pause();
    Robot.drivetrain.tankDrive(0.0,0.0);
    Robot.isClimbEnabledLevel2 = true;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
