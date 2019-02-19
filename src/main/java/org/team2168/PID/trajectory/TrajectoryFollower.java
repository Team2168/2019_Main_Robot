
//FIXME Outputs neg val instead of pos

package org.team2168.PID.trajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryFollower {

	private double kp_;
	private double ki_; // Not currently used, but might be in the future.
	private double kd_;
	private double kv_;
	private double ka_;
	private double last_error_;
	private double current_heading = 0;
	private int current_segment;
	private Trajectory profile_;
	public String name;
	private double errsum;
	private double int_d_term;
	private double lastDeriv;

	public TrajectoryFollower(String name) {
		this.name = name;
	}

	public void configure(double kp, double ki, double kd, double kv, double ka) {
		kp_ = kp;
		ki_ = ki;
		kd_ = kd;
		kv_ = kv;
		ka_ = ka;
	}

	public void reset() {
		last_error_ = 0.0;
		current_segment = 0;
	}

	public void setTrajectory(Trajectory profile) {
		profile_ = profile;
	}

	public double calculate(double distance_so_far) {

		if (current_segment < profile_.getNumSegments()) {
			Trajectory.Segment segment = profile_.getSegment(current_segment);
			double error = segment.pos - distance_so_far;

			// double output = kp_ * error + kd_ * ((error - last_error_)
			// / (2*segment.dt) - segment.vel) + (kv_ * segment.vel
			// + ka_ * segment.acc);

			// calculate derivative gain d/dt
			double executionTime = 0.02; // time

			// prevent divide by zero error, by disabiling deriv term
			// if execution time is zero.

			double diff = 0;
			if (executionTime > 0)
				diff = (error - last_error_) / executionTime; // delta
			else
				diff = 0;

			double deriv = kd_ * diff - segment.vel;

			// integral
			errsum = errsum + (last_error_ * executionTime);
			double integ = ki_ * errsum; // final integral term

			// proportional term
			double prop = kp_ * error;

			// calculate new control output based on filtering
			double output = prop + integ + deriv + kv_ * segment.vel + ka_ * segment.acc;

			// integral anti-windup control via clamping
			if ((output > 1 || output < -1) && (Math.signum(output) == Math.signum(ki_ * error))) {
				errsum = 0;
				integ = ki_ * errsum;
				output = prop + integ + deriv + kv_ * segment.vel + ka_ * segment.acc;
				;

			}

			// Saturation
			if (output > 1)
				output = 1;
			if (output < -1)
				output = -1;

			last_error_ = error;
			current_heading = segment.heading;
			current_segment++;
			SmartDashboard.putNumber(name + "FollowerSensor", distance_so_far);
			SmartDashboard.putNumber(name + "FollowerGoal", segment.pos);
			SmartDashboard.putNumber(name + "FollowerError", error);
			System.out.println(name + ":" + output + "/Output");
			return output;
		} else {
			return 0;
		}
	}

	public double getHeading() {
		return current_heading;
	}

	public boolean isFinishedTrajectory() {
		return current_segment >= profile_.getNumSegments();
	}

	public int getCurrentSegment() {
		return current_segment;
	}

	public int getNumSegments() {
		return profile_.getNumSegments();
	}
}
