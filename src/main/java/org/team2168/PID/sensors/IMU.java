package org.team2168.PID.sensors;

import org.team2168.Robot;

/**
 * 
 * @author Kevin Harrilal, Team 2168 Aluminum Falcons
 * 
 */
public class IMU implements PIDSensorInterface {

	PIDSensorInterface leftEncoder;
	PIDSensorInterface rightEncoder;
	double wheelBase;

	public IMU(PIDSensorInterface leftEncoder, PIDSensorInterface rightEncoder, double wheelBase) {

		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.wheelBase = wheelBase;

	}

	// public static double getAccPitch() {
	//
	// double X = Robot.accel.getX();
	// double Y = Robot.accel.getY();
	// double Z = Robot.accel.getZ();
	//
	//
	// return Math.atan2(Y,Z) *180 /Math.PI;
	// }
	//
	// public static double getAccRoll()
	// {
	//
	//// double X = Robot.accel.getX();
	//// double Y = Robot.accel.getY();
	//// double Z = Robot.accel.getZ();
	//
	// return Math.atan2(-X, Math.sqrt(Y*Y + Z*Z)) * 180/Math.PI;
	//
	// }

	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		leftEncoder.reset();
		rightEncoder.reset();

	}

	@Override
	public double getPos() {
		// TODO Auto-generated method stub
		return (leftEncoder.getPos() + rightEncoder.getPos()) / 2;
	}

}
