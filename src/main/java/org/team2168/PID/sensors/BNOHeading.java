// // package org.team2168.PID.sensors;

// import org.team2168.utils.BNO055;

// import edu.wpi.first.wpilibj.I2C;

// /**
//  * A wrapper for the BNO055 class which implements the PIDSensorInterface for
//  * the heading of the IMU sensor.
//  * 
//  * @author James
//  */
// public class BNOHeading implements PIDSensorInterface {

// 	BNO055 instance = null;

// 	/**
// 	 * Get an instance of the IMU object.
// 	 * 
// 	 * @param mode
// 	 *            the operating mode to run the sensor in.
// 	 * @param port
// 	 *            the physical port the sensor is plugged into on the roboRio
// 	 * @param address
// 	 *            the address the sensor is at (0x28 or 0x29)
// 	 */
// 	public BNOHeading(BNO055.opmode_t mode, BNO055.vector_type_t vectorType, I2C.Port port, byte address) {
// 		instance = BNO055.getInstance(mode, vectorType, port, address);
// 	}

// 	/**
// 	 * Get an instance of the IMU object plugged into the onboard I2C header. Using
// 	 * the default address (0x28)
// 	 * 
// 	 * @param mode
// 	 *            the operating mode to run the sensor in.
// 	 * @param vectorType
// 	 *            the format the position vector data should be returned in (if you
// 	 *            don't know use VECTOR_EULER).
// 	 */
// 	public BNOHeading(BNO055.opmode_t mode, BNO055.vector_type_t vectorType) {
// 		instance = BNO055.getInstance(mode, vectorType);
// 	}

// 	public BNOHeading(BNO055 instance) {
// 		this.instance = instance;
// 	}

// 	/**
// 	 * Returns zero, this isn't a rate sensor.
// 	 * 
// 	 * @return 0
// 	 */
// 	@Override
// 	public double getRate() {
// 		return 0;
// 	}

// 	/**
// 	 * Reset the heading to zero.
// 	 */
// 	@Override
// 	public void reset() {
// 		// TODO Auto-generated method stub
// 		instance.reset();
// 	}

// 	/**
// 	 * 
// 	 * @return the relative heading of the sensor.
// 	 */
// 	@Override
// 	public double getPos() {
// 		return instance.getHeading();
// 	}

// }
