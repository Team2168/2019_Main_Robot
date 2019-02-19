//FIXME: Re-add Radians to GyroSpi

package org.team2168.commands.drivetrain.PIDCommands;

import org.team2168.Robot;
import org.team2168.PID.trajectory.Trajectory;
import org.team2168.PID.trajectory.TrajectoryFollower;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj.command.Command;


public class DriveTrajectoryPath extends Command {

	Trajectory trajectory;
	TrajectoryFollower followerLeft = new TrajectoryFollower("left");
	TrajectoryFollower followerRight = new TrajectoryFollower("right");
	double direction;
	double heading;
	double kTurn = -3.0/80.0;

	public DriveTrajectoryPath(Trajectory leftProfile, Trajectory rightProfile, double direction, double heading){
		requires(Robot.drivetrain);
		followerLeft.configure(0.029, 0,0,0,0);
		followerRight.configure(0.029, 0,0,0,0);

		reset();
		followerLeft.setTrajectory(leftProfile);
		followerRight.setTrajectory(rightProfile);
		this.direction = direction;
		this.heading = heading;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		reset();

	}

	// Called repeatedly when this Command is scheduled to run

	protected void execute() {

		System.out.println(followerRight.getNumSegments());

		if (followerLeft.isFinishedTrajectory())
			Robot.drivetrain.tankDrive(0.0, 0.0);
		else
		{
			double distanceL = direction * Robot.drivetrain.getLeftPosition(); //feet
			double distanceR = direction * Robot.drivetrain.getRightPosition(); //feet

			System.out.println("Distance L:" + distanceL);
			System.out.println("Distance R:" + distanceR);

			double speedRight = direction * followerRight.calculate(distanceR);
			double speedLeft = direction * followerLeft.calculate(distanceL);


			//double goalHeading = followerLeft.getHeading();
			double goalHeading = 0;
			//double observedHeading = Robot.drivetrain.gyroSPI.getAngleRadians(); //radians

			double observedHeading = 0;
			double angleDiffRads = Util.getDifferenceInAngleRadians(observedHeading, goalHeading);
			double angleDiff = Math.toDegrees(angleDiffRads);

			//System.out.println("Speed Left:" + speedLeft);

			//System.out.println("Speed Right:" + speedRight);

			double turn = kTurn * angleDiff;
			Robot.drivetrain.tankDrive(-speedLeft, -speedRight);
		}

	}

	// Make this return true when this Command no longer needs to run execute()

	protected boolean isFinished() {
		return followerLeft.isFinishedTrajectory();
	}

	// Called once after isFinished returns true

	protected void end() {

	}

	//delete me
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run

	protected void interrupted() {
		end();
	}

	public void reset() {
		followerLeft.reset();
		followerRight.reset();
		Robot.drivetrain.resetPosition();
	}
}
