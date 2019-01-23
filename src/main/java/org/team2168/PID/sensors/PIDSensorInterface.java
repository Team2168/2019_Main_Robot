package org.team2168.PID.sensors;

public interface PIDSensorInterface {
	/**
	 *
	 * @return the current rate of the sensor in nominal units of the sensor.
	 *         Remember when using this method with a PID controller to ensure that
	 *         the setPoints of the PID controller are in the same unit as returned
	 *         by this function.
	 */
	public double getRate();

	/**
	 * Resets the rate of the sensor to zero and clears any accumulators/counters to
	 * zero.
	 */
	public void reset();

	/**
	 *
	 * @return the current Position of the sensor in nominal units of the sensor.
	 *         Remember when using this method with a PID controller to ensure that
	 *         the setPoints of the PID controller are in the same unit as returned
	 *         by this function.
	 */
	public double getPos();

	/**
	 * Enumeration of Speed Units to be returned by a Speed Sensor
	 * 
	 * @author HarrilalEngineering
	 *
	 */
	public static class SpeedReturnType {
		/**
		 * The integer value representing this enumeration
		 */
		static final int IPS_val = 0;
		static final int RPM_val = 1;
		static final int FPS_val = 2;
		static final int PERIOD_val = 3;
		final int value;
		/**
		 * Inch Per Second
		 */
		public static final SpeedReturnType IPS = new SpeedReturnType(IPS_val);
		/**
		 * Rotation Per Minute
		 */
		public static final SpeedReturnType RPM = new SpeedReturnType(RPM_val);
		/**
		 * Feet per Second
		 */
		public static final SpeedReturnType FPS = new SpeedReturnType(FPS_val);
		/**
		 * Period in Seconds
		 */
		public static final SpeedReturnType PERIOD = new SpeedReturnType(PERIOD_val);

		private SpeedReturnType(int value) {
			this.value = value;
		}
	}

	/**
	 * Enumeration of Position Units to be returned by a Position Sensor
	 * 
	 * @author HarrilalEngineering
	 *
	 */
	public static class PositionReturnType {
		static final int TICKS_val = 0;
		static final int INCH_val = 1;
		static final int DEGREE_val = 2;
		static final int RADIANS_val = 3;
		static final int FEET_val = 4;
		public final int value;

		/**
		 * Ticks
		 */
		public static final PositionReturnType TICKS = new PositionReturnType(TICKS_val);
		/**
		 * Inch traveled
		 */
		public static final PositionReturnType INCH = new PositionReturnType(INCH_val);
		/**
		 * Degrees Rotated
		 */
		public static final PositionReturnType DEGREE = new PositionReturnType(DEGREE_val);
		/**
		 * Radians Rotated
		 */
		public static final PositionReturnType RADIANS = new PositionReturnType(RADIANS_val);

		/**
		 * Feet traveled
		 */
		public static final PositionReturnType FEET = new PositionReturnType(FEET_val);

		private PositionReturnType(int value) {
			this.value = value;
		}
	}

}