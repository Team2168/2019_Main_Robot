/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.commands.monkeyBarPivot.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.OneDimensionalMotionProfilingNoConstraint;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMonkeyBarPivotPIDPathAutoClimb extends Command {

  private double[] pos;
  private double[] vel;
  private double[] accel;
  

  private double setPoint;
  OneDimensionalMotionProfilingNoConstraint motion;
	int counter;
  double ff_term = 0.2;

  private double maxSpeed;
  private double minSpeed;
  private double error = 2; //Rotational degree error, 0 never ends


  public final double MAX_VEL = 110;
  public final double MAX_ACCEL = 110;
  public final double MAX_JERK = 1000;


  private double time;
  private double start;
  private double end;



  private DriveMonkeyBarPivotPIDPathAutoClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.monkeyBarPivot);
    this.setPoint = Robot.monkeyBarPivot.monkeyBarPivotController.getSetPoint();
    this.maxSpeed = 1.0;
    this.minSpeed = 0.15;

    SmartDashboard.putNumber("FF_term_MB", ff_term);
    
  }

  public DriveMonkeyBarPivotPIDPathAutoClimb(double end, double time)
  {
    this();
    this.start = 0;
    this.end = end;
    this.time = time;
  }

  public DriveMonkeyBarPivotPIDPathAutoClimb(double end, double time, boolean absolute)
  {
    this();
    if(absolute)
      this.start = Robot.monkeyBarPivot.getRightPotPos();
    else
      this.start = 0;
    this.end = end;
    this.time = time;
  }
 
  public DriveMonkeyBarPivotPIDPathAutoClimb(double start, double end, double time)
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
    
    Robot.monkeyBarPivot.monkeyBarPivotController.reset();

    // Robot.monkeyBarPivot.monkeyBarPivotController.setpGain(RobotMap.MB_PIVOT_P);
    // Robot.monkeyBarPivot.monkeyBarPivotController.setiGain(RobotMap.MB_PIVOT_I);
    // Robot.monkeyBarPivot.monkeyBarPivotController.setdGain(RobotMap.MB_PIVOT_D);
    Robot.monkeyBarPivot.monkeyBarPivotController.setSetPoint(this.pos);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMaxPosOutput(maxSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMaxNegOutput(-maxSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMinPosOutput(minSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setMinNegOutput(-minSpeed);
    Robot.monkeyBarPivot.monkeyBarPivotController.setAcceptErrorDiff(error);

    Robot.monkeyBarPivot.monkeyBarPivotController.Enable();

    counter = 0;

    System.out.print("Lenght: "+ pos.length);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    ff_term = SmartDashboard.getNumber("FF_term_MB", ff_term);


    if (counter < pos.length)
    {
      double pidSpeed = Robot.monkeyBarPivot.monkeyBarPivotController.getControlOutput();
      double ff_Speed = (ff_term  * vel[counter]) / (Robot.pdp.getBatteryVoltage());
      Robot.monkeyBarPivot.driveRotateBarMotors(ff_Speed+pidSpeed);
      System.out.println(ff_Speed+pidSpeed);
    }
    else
      Robot.monkeyBarPivot.driveRotateBarMotors(0.0);


      counter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (counter >= pos.length) || (Robot.monkeyBarPivot.getRightPotPos() < pos[pos.length-1]+1 && Robot.monkeyBarPivot.getRightPotPos() > pos[pos.length-1]) ;
    return (Robot.monkeyBarPivot.getRightPotPos() < pos[pos.length-1]+1 && Robot.monkeyBarPivot.getRightPotPos() > pos[pos.length-1] || Robot.oi.driverJoystick.getLeftStickRaw_X() < -0.1 || Robot.oi.driverJoystick.getLeftStickRaw_X() > 0.1);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.monkeyBarPivot.monkeyBarPivotController.Pause();
    Robot.monkeyBarPivot.driveRotateBarMotors(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
