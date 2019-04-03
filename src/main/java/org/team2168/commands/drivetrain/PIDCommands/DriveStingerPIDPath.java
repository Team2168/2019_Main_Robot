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


public class DriveStingerPIDPath extends Command {

  private double[] pos;
  private double[] vel;
  private double[] accel;
  

  private double setPoint;
  OneDimensionalMotionProfilingNoConstraint motion;
	int counter;
  double ff_term = 0.30;

  private double maxSpeed;
  private double minSpeed;
  private double error = 2; //Rotational degree error, 0 never ends



  public DriveStingerPIDPath() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    this.setPoint = Robot.drivetrain.leftPosController.getSetPoint();
    this.maxSpeed = 1.0;
    this.minSpeed = 0.0;

    //SmartDashboard.putNumber("FF_term_MB", 0);
    
  }

  public DriveStingerPIDPath(double setPoint)
  {
    this();
    this.setPoint = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {

    motion = new OneDimensionalMotionProfilingNoConstraint(6.0,5); 
    
    this.pos = motion.pos;
    this.vel = motion.vel;

    Robot.drivetrain.resetPosition();
    Robot.drivetrain.leftPosController.reset();
    Robot.drivetrain.leftPosController.setpGain(RobotMap.MB_PIVOT_P);
    Robot.drivetrain.leftPosController.setiGain(RobotMap.MB_PIVOT_I);
    Robot.drivetrain.leftPosController.setdGain(RobotMap.MB_PIVOT_D);
    Robot.drivetrain.leftPosController.setSetPoint(this.pos);
    Robot.drivetrain.leftPosController.setMaxPosOutput(maxSpeed);
    Robot.drivetrain.leftPosController.setMaxNegOutput(-maxSpeed);
    Robot.drivetrain.leftPosController.setMinPosOutput(minSpeed);
    Robot.drivetrain.leftPosController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.leftPosController.setAcceptErrorDiff(error);
    Robot.drivetrain.leftPosController.Enable();

    Robot.drivetrain.rightPosController.reset();
    Robot.drivetrain.rightPosController.setpGain(RobotMap.MB_PIVOT_P);
    Robot.drivetrain.rightPosController.setiGain(RobotMap.MB_PIVOT_I);
    Robot.drivetrain.rightPosController.setdGain(RobotMap.MB_PIVOT_D);
    Robot.drivetrain.rightPosController.setSetPoint(this.pos);
    Robot.drivetrain.rightPosController.setMaxPosOutput(maxSpeed);
    Robot.drivetrain.rightPosController.setMaxNegOutput(-maxSpeed);
    Robot.drivetrain.rightPosController.setMinPosOutput(minSpeed);
    Robot.drivetrain.rightPosController.setMinNegOutput(-minSpeed);
    Robot.drivetrain.rightPosController.setAcceptErrorDiff(error);
    Robot.drivetrain.rightPosController.Enable();

    counter = 0;

    System.out.print("Lenght: "+ pos.length);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //ff_term = SmartDashboard.getNumber("FF_term_MB", 0);


    if (counter < pos.length)
    {
      double pidLSpeed = Robot.drivetrain.leftPosController.getControlOutput();
      double pidRSpeed = Robot.drivetrain.rightPosController.getControlOutput();
      double ff_Speed = (ff_term  * vel[counter]) / (Robot.pdp.getBatteryVoltage());
      Robot.drivetrain.tankDrive(ff_Speed+pidLSpeed,ff_Speed+pidRSpeed);
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
    Robot.drivetrain.leftPosController.Pause();
    Robot.drivetrain.rightPosController.Pause();
    Robot.drivetrain.tankDrive(0.0,0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
