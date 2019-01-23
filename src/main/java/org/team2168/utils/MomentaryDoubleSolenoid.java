package org.team2168.utils;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import java.util.Timer;
import java.util.TimerTask;

/**
 * This class keeps the output of a double solenoid valve energized for a
 * specified duration after it's state has changed. After the energized duration
 * has elapsed, the solenoid is de-energized.
 * 
 * This is done to prevent damage to the solenoid valves, which have been
 * observed to become quite hot after extended use.
 *
 * @author james@team2168.org
 *
 */
public class MomentaryDoubleSolenoid extends DoubleSolenoid {
	private Timer latchTimer;
	private long latchTime; // time to hold the output energized
	private boolean energized = false; // flag to keep track of an active delay

	/**
	 * A double solenoid which leaves its output energized for the specified delay
	 * time period (ms).
	 * 
	 * @param forwardChannel
	 *            The forward channel on the module to control.
	 * @param reverseChannel
	 *            The reverse channel on the module to control.
	 * @param delay
	 *            The time in ms to leave the solenoid outputs energized.
	 */
	public MomentaryDoubleSolenoid(final int forwardChannel, final int reverseChannel, long delay) {
		super(forwardChannel, reverseChannel);
		latchTime = delay;
	}

	/**
	 * A double solenoid which leaves its output energized for the default time
	 * period (1s).
	 * 
	 * @param forwardChannel
	 *            The forward channel on the module to control.
	 * @param reverseChannel
	 *            The reverse channel on the module to control.
	 */
	public MomentaryDoubleSolenoid(final int forwardChannel, final int reverseChannel) {
		this(forwardChannel, reverseChannel, (long) 1000);
	}

	/**
	 * A double solenoid which leaves its output energized for the specified delay
	 * time period (ms).
	 * 
	 * @param moduleNumber
	 *            The module position the relay module is installed in on the CRIO.
	 * @param forwardChannel
	 *            The forward channel on the module to control.
	 * @param reverseChannel
	 *            The reverse channel on the module to control.
	 * @param delay
	 *            The time in ms to leave the solenoid outputs energized.
	 */
	public MomentaryDoubleSolenoid(final int moduleNumber, final int forwardChannel, final int reverseChannel,
			long delay) {
		super(moduleNumber, forwardChannel, reverseChannel);
		latchTime = delay;
	}

	/**
	 * A double solenoid which leaves its output energized for the default time
	 * period (1s).
	 * 
	 * @param moduleNumber
	 *            The module position the relay module is installed in on the CRIO.
	 * @param forwardChannel
	 *            The forward channel on the module to control.
	 * @param reverseChannel
	 *            The reverse channel on the module to control.
	 */
	public MomentaryDoubleSolenoid(final int moduleNumber, final int forwardChannel, final int reverseChannel) {
		this(moduleNumber, forwardChannel, reverseChannel, (long) 1000);
	}

	/**
	 * Sets the state of the solenoid.
	 * 
	 * @param value
	 */
	public void set(final DoubleSolenoid.Value value) {
		// If there is currently an active timerTask, kill it
		if (energized)
			latchTimer.cancel();

		energized = true;
		// Don't schedule a callback if the solenoid is explicitly set to off.
		if (value != Value.kOff) {
			latchTimer = new Timer();
			latchTimer.schedule(new LatchTimerTask(), latchTime);
		}
		super.set(value);
	}

	/**
	 * This method will be called after the specified amount of time to leave the
	 * solenoid energized has elapsed. If changes the output state of the solenoid,
	 * and cleans up the associated TimerTask.
	 */
	private void turnOff() {
		super.set(Value.kOff);
		latchTimer.cancel();
		energized = false;
	}

	/**
	 * This task's run() method will be executed at the scheduled time specified in
	 * the constructor, at which point the solenoid output will be de-energized.
	 * 
	 * @author James
	 *
	 */
	private class LatchTimerTask extends TimerTask {
		public void run() {
			turnOff();
		}
	}
}