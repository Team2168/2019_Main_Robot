// package org.team2168.PID.sensors;

// /*----------------------------------------------------------------------------*/
// /* Copyright (c) FIRST 2008-2012. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// import edu.wpi.first.wpilibj.AccumulatorResult;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PIDSource;
// import edu.wpi.first.wpilibj.PIDSourceType;
// import edu.wpi.first.wpilibj.SensorBase;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
// import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
// import edu.wpi.first.wpilibj.tables.ITable;
// import edu.wpi.first.wpilibj.util.BoundaryException;

// /**
//  * Re-written to support RoboRio. 8/19/2014 Kevin Harrilal
//  * 
//  * Use a rate gyro to return the robots heading relative to a starting position.
//  * The Gyro class tracks the robots heading based on the starting position. As
//  * the robot rotates the new heading is computed by integrating the rate of
//  * rotation returned by the sensor. When the class is instantiated, it does a
//  * short calibration routine where it samples the gyro while at rest to
//  * determine the default offset. This is subtracted from each sample to
//  * determine the heading.
//  * 
//  * Modified to allow external calls in to initialize the gyro, ala cheezygyro.
//  * This allows a the robot to re-zero the gyro if it was drifting before the
//  * match has started. This could happen if the robot was being moved around
//  * shortly after power on (during the first gyro init).
//  * 
//  * Also modified to prevent this class from blocking execution while
//  * initializing the gyro.
//  * 
//  * @author James@team2168.org
//  */
// public class FalconGyro extends SensorBase implements PIDSensorInterface, PIDSource, LiveWindowSendable {

// 	static final int kOversampleBits = 10;
// 	static final int kAverageBits = 0;
// 	static final double kSamplesPerSecond = 50.0;
// 	static final double kCalibrationSampleTime = 5.0;
// 	static final double kDefaultVoltsPerDegreePerSecond = 0.007;
// 	protected AnalogInput m_analog;
// 	double m_voltsPerDegreePerSecond;
// 	double m_offset;
// 	int m_center;
// 	boolean m_channelAllocated;
// 	AccumulatorResult result;
// 	PIDSourceType m_pidSource;

// 	/**
// 	 * Initialize the gyro. Calibrate the gyro by running for a number of samples
// 	 * and computing the center value for this part. Then use the center value as
// 	 * the Accumulator center value for subsequent measurements. It's important to
// 	 * make sure that the robot is not moving while the centering calculations are
// 	 * in progress, this is typically done when the robot is first turned on while
// 	 * it's sitting at rest before the competition starts.
// 	 */
// 	private void initGyro() {
// 		result = new AccumulatorResult();
// 		if (m_analog == null) {
// 			System.out.println("Null m_analog");
// 		}
// 		m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
// 		m_analog.setAverageBits(kAverageBits);
// 		m_analog.setOversampleBits(kOversampleBits);
// 		double sampleRate = kSamplesPerSecond * (1 << (kAverageBits + kOversampleBits));
// 		AnalogInput.setGlobalSampleRate(sampleRate);

// 		Timer.delay(1.0);

// 		reInitGyro();

// 		// setPIDSourceParameter(PIDSourceParameter.kAngle);

// 		LiveWindow.addSensor("Gyro", m_analog.getChannel(), this);
// 	}

// 	/**
// 	 * Initialize the gyro. This method will block for the calibration period or
// 	 * until the match has started, whichever happens first. If this method is
// 	 * called during a match (the robot is enabled), the previous calibration value
// 	 * will be kept.
// 	 */
// 	public void reInitGyro() {
// 		double startTime = Timer.getFPGATimestamp();

// 		// Don't bother re-initializing the gyro if the match has already started
// 		if (DriverStation.getInstance().isDisabled()) {
// 			m_analog.initAccumulator();
// 			// make sure we wait long enough to accumulate at least one sample
// 			Timer.delay(0.025);

// 			// Attempt to calibrate the gyro. Delay until we have waited for the
// 			// length of the calibration period or a match has started,
// 			// whichever happens first.
// 			while ((Timer.getFPGATimestamp() < startTime + kCalibrationSampleTime)
// 					&& DriverStation.getInstance().isDisabled()) {
// 				Timer.delay(0.005);
// 			}

// 			m_analog.getAccumulatorOutput(result);

// 			// Determine the new zero, where value is the accumulated analog data
// 			// over the time specified (count).
// 			m_center = (int) ((double) result.value / (double) result.count + .5);

// 			m_offset = ((double) result.value / (double) result.count) - (double) m_center;

// 			m_analog.setAccumulatorCenter(m_center);

// 			m_analog.setAccumulatorDeadband(0);

// 			m_analog.resetAccumulator();
// 		}
// 	}

// 	/**
// 	 * Gyro constructor with only a channel.
// 	 * 
// 	 * Use the default analog module slot.
// 	 * 
// 	 * @param channel
// 	 *            The analog channel the gyro is connected to.
// 	 */
// 	public FalconGyro(int channel) {
// 		m_analog = new AnalogInput(channel);
// 		m_channelAllocated = true;
// 		initGyro();
// 	}

// 	/**
// 	 * Gyro constructor with a precreated analog channel object. Use this
// 	 * constructor when the analog channel needs to be shared. There is no reference
// 	 * counting when an AnalogChannel is passed to the gyro.
// 	 * 
// 	 * @param channel
// 	 *            The AnalogChannel object that the gyro is connected to.
// 	 */
// 	public FalconGyro(AnalogInput channel) {
// 		m_analog = channel;
// 		if (m_analog == null) {
// 			System.err.println("Analog channel supplied to Gyro constructor is null");
// 		} else {
// 			m_channelAllocated = false;
// 			initGyro();
// 		}
// 	}

// 	/**
// 	 * Reset the gyro. Resets the gyro to a heading of zero. This can be used if
// 	 * there is significant drift in the gyro and it needs to be recalibrated after
// 	 * it has been running.
// 	 */
// 	@Override
// 	public void reset() {
// 		if (m_analog != null) {
// 			m_analog.resetAccumulator();
// 		}
// 	}

// 	/**
// 	 * Delete (free) the accumulator and the analog components used for the gyro.
// 	 */
// 	public void free() {
// 		if (m_analog != null && m_channelAllocated) {
// 			m_analog.free();
// 		}
// 		m_analog = null;
// 	}

// 	/**
// 	 * Return the actual angle in degrees that the robot is currently facing.
// 	 * 
// 	 * The angle is based on the current accumulator value corrected by the
// 	 * oversampling rate, the gyro type and the A/D calibration values. The angle is
// 	 * continuous, that is can go beyond 360 degrees. This make algorithms that
// 	 * wouldn't want to see a discontinuity in the gyro output as it sweeps past 0
// 	 * on the second time around.
// 	 * 
// 	 * @return the current heading of the robot in degrees. This heading is based on
// 	 *         integration of the returned rate from the gyro.
// 	 */
// 	public double getAngle() {
// 		if (m_analog == null) {
// 			return 0.0;
// 		} else {
// 			m_analog.getAccumulatorOutput(result);

// 			long value = result.value - (long) (result.count * m_offset);

// 			double scaledValue = value * 1e-9 * m_analog.getLSBWeight() * (1 << m_analog.getAverageBits())
// 					/ (AnalogInput.getGlobalSampleRate() * m_voltsPerDegreePerSecond);

// 			return scaledValue;
// 		}
// 	}

// 	/**
// 	 * Return the rate of rotation of the gyro
// 	 * 
// 	 * The rate is based on the most recent reading of the gyro analog value
// 	 * 
// 	 * @return the current rate in degrees per second
// 	 */
// 	@Override
// 	public double getRate() {
// 		if (m_analog == null) {
// 			return 0.0;
// 		} else {
// 			return (m_analog.getAverageValue() - ((double) m_center + m_offset)) * 1e-9 * m_analog.getLSBWeight()
// 					/ ((1 << m_analog.getOversampleBits()) * m_voltsPerDegreePerSecond);
// 		}
// 	}

// 	/**
// 	 * Set the gyro type based on the sensitivity. This takes the number of
// 	 * volts/degree/second sensitivity of the gyro and uses it in subsequent
// 	 * calculations to allow the code to work with multiple gyros.
// 	 * 
// 	 * @param voltsPerDegreePerSecond
// 	 *            The type of gyro specified as the voltage that represents one
// 	 *            degree/second.
// 	 */
// 	public void setSensitivity(double voltsPerDegreePerSecond) {
// 		m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
// 	}

// 	/**
// 	 * Set which parameter of the encoder you are using as a process control
// 	 * variable. The Gyro class supports the rate and angle parameters
// 	 * 
// 	 * @param type
// 	 *            the type of output to provide to the PID controller
// 	 *            (rate/displacement).
// 	 */
// 	public void setPIDSourceType(PIDSourceType type) {
// 		m_pidSource = type;
// 	}

// 	/**
// 	 * Get the PID source type.
// 	 * 
// 	 * @param test
// 	 * @return the PID source type (rate/displacement)
// 	 */
// 	public PIDSourceType getPIDSourceType() {
// 		return m_pidSource;
// 	}

// 	/**
// 	 * Get the angle of the gyro for use with PIDControllers
// 	 * 
// 	 * @return the current angle according to the gyro
// 	 */
// 	public double pidGet() {
// 		switch (m_pidSource) {
// 		case kRate:
// 			return getRate();
// 		case kDisplacement:
// 			return getAngle();
// 		default:
// 			return 0.0;
// 		}
// 	}

// 	/*
// 	 * Live Window code, only does anything if live window is activated.
// 	 */
// 	public String getSmartDashboardType() {
// 		return "Gyro";
// 	}

// 	private ITable m_table;

// 	/**
// 	 * {@inheritDoc}
// 	 */
// 	public void initTable(ITable subtable) {
// 		m_table = subtable;
// 		updateTable();
// 	}

// 	/**
// 	 * {@inheritDoc}
// 	 */
// 	public ITable getTable() {
// 		return m_table;
// 	}

// 	/**
// 	 * {@inheritDoc}
// 	 */
// 	public void updateTable() {
// 		if (m_table != null) {
// 			m_table.putNumber("Value", getAngle());
// 		}
// 	}

// 	/**
// 	 * {@inheritDoc}
// 	 */
// 	public void startLiveWindowMode() {
// 	}

// 	/**
// 	 * {@inheritDoc}
// 	 */
// 	public void stopLiveWindowMode() {
// 	}

// 	@Override
// 	public double getPos() {
// 		// TODO Auto-generated method stub
// 		return getAngle();
// 	}
// }
