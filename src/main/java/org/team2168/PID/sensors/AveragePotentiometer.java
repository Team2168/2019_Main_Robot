package org.team2168.PID.sensors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import org.team2168.utils.LinearInterpolator;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class builds on the basic AnalogInput class and makes it specifically
 * for a potentiometer. It takes in the values for max, zero, and min voltage
 * and angles and creates a linear interpolator to calculate angle for use with
 * PID controllers.
 * 
 * @author Wen Baid
 *
 */
public class AveragePotentiometer implements PIDSensorInterface
{

	private int averagorSize;
	private double[] averagorArray;
	private int arrayPos = 0; // Next array position to put values to be
	// averaged

	private double minVoltage;
	private double maxVoltage;

	private boolean isCan = false;

	private TalonSRX motor;

	AnalogInput potentiometer;
	LinearInterpolator interpolator;
	double[][] range;

	public AveragePotentiometer(int channel, double zeroVoltage, double zeroAngle, double maxVoltage, double maxAngle,
			int averageN)
	{

		potentiometer = new AnalogInput(channel);

		double[][] tempRange = { { zeroVoltage, zeroAngle
				}, { maxVoltage, maxAngle
				}
		};

		this.range = tempRange;

		interpolator = new LinearInterpolator(this.range);

		this.averagorSize = averageN;
		this.averagorArray = new double[averagorSize];

		this.minVoltage = zeroVoltage;
		this.maxVoltage = maxVoltage;

	}

	private AveragePotentiometer(double zeroVoltage, double zeroAngle, double maxVoltage, double maxAngle, int averageN)
	{
		double[][] tempRange = { { zeroVoltage, zeroAngle
				}, { maxVoltage, maxAngle
				}
		};

		this.range = tempRange;

		interpolator = new LinearInterpolator(this.range);

		this.averagorSize = averageN;
		this.averagorArray = new double[averagorSize];

		this.minVoltage = zeroVoltage;
		this.maxVoltage = maxVoltage;

	}

	// public AveragePotentiometer(int channel, double minVoltage, double minAngle,
	// double zeroVoltage, double zeroAngle,
	// double maxVoltage, double maxAngle, int averageN) {

	// this(minVoltage, minAngle, zeroVoltage, zeroAngle, maxVoltage, maxAngle,
	// averageN);

	// potentiometer = new AnalogInput(channel);
	// }

	public AveragePotentiometer(TalonSRX motor, double zeroVoltage, double zeroAngle, double maxVoltage,
			double maxAngle, int averageN)
	{
		this(zeroVoltage, zeroAngle, maxVoltage, maxAngle, averageN);

		this.motor = motor;

		this.motor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

		this.isCan = true;
	}

	@Override
	public double getRate()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void reset()
	{
		// TODO Auto-generated method stub

	}

	@Override
	public double getPos()
	{
		// if(isCan)
		// {
		// return this.motor.getSensorCollection().getAnalogIn();
		// }
		// else
		// {
		// return interpolator.interpolate(potentiometer.getVoltage());
		// }

		return interpolator.interpolate(this.getRawPos());

	}

	// 19 Feb: Analog input returns number between 0 and 1023 so we scaled to be
	// between 0 and 5 V
	public double getRawPos()
	{
		if (isCan)
		{
			return this.motor.getSensorCollection().getAnalogInRaw() * (5.0 / 1023.0);
		}
		else
		{
			return potentiometer.getVoltage();
		}
	}

	public boolean isAtUpperLimit()
	{
		return getRawPos() >= maxVoltage;
	}

	public boolean isAtLowerLimit()
	{
		return getRawPos() <= minVoltage;
	}

	/**
	 * returns (gets) Average of last n values sent, as name says.
	 *
	 * @return the Average
	 */
	public synchronized double getAverage()
	{
		double sum = 0;

		for (int i = 0; i < averagorSize; i++)
			sum += averagorArray[i];

		// System.out.println("Average: " + Arrays.toString(averagorArray) + (sum /
		// averagorSize));
		return sum / averagorSize;
	}

	/**
	 * puts data in to array to be averaged, hence the class name and method name.
	 * Its like magic but cooler.
	 *
	 * @param value the value being inserted into the array to be averaged.
	 */

	public synchronized void putData(double value)
	{

		averagorArray[arrayPos] = value;
		arrayPos++;

		if (arrayPos >= averagorSize) // Is equal or greater to averagorSize
			// because array is zero indexed. Rolls
			// over index position.
			arrayPos = 0;
	}

}
