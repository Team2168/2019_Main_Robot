package org.team2168.subsystems;

import org.team2168.RobotMap;
import org.team2168.commands.pneumatics.StartCompressor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls the compressor and reads pressure sensor
 * 
 * @author Ben Waid
 */
public class Pneumatics extends Subsystem {

	private Compressor compressor;
	private AnalogInput pressureSensor;

	private static Pneumatics instance = null;

	/**
	 * Private constructor for the Pneumatics subsystem
	 */
	private Pneumatics() {
		compressor = new Compressor();
		pressureSensor = new AnalogInput(RobotMap.PRESSURE_SENSOR);
	}

	/**
	 * Singleton constructor for Pneumatics subsystem
	 * 
	 * @return singleton instance of Pneumatics subsystem
	 */
	public static Pneumatics getInstance() {
		if (instance == null)
			instance = new Pneumatics();
		return instance;
	}

	/**
	 * Starts the compressor
	 */
	public void startCompressor() {
		compressor.start();
	}

	/**
	 * Gets the voltage from the pressure sensor
	 * 
	 * @return double voltage
	 */
	public double getVoltage() {
		return pressureSensor.getVoltage();
	}

	public double getPSI() {
		double pressure = (37.5 * getVoltage()) - 18.75;
		return pressure;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new StartCompressor());
	}
}
