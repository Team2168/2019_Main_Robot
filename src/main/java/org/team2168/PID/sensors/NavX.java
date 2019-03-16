/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.PID.sensors;

import com.kauailabs.navx.frc.AHRS;

import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class NavX implements PIDSensorInterface{

    public AHRS ahrs;

    public NavX()
    {
        ahrs = new AHRS(SPI.Port.kMXP);

        ConsolePrinter.putNumber("NAVX Pitch", () -> { return (double)ahrs.getPitch();}, true, false);
        ConsolePrinter.putNumber("NAVX Yaw", () -> { return (double)ahrs.getYaw();}, true, false);
        ConsolePrinter.putNumber("NAVX Roll", () -> { return (double)ahrs.getRoll();}, true, false);
    }



    /**
	 *
	 * @return the current rate of the sensor in nominal units of the sensor.
	 *         Remember when using this method with a PID controller to ensure that
	 *         the setPoints of the PID controller are in the same unit as returned
	 *         by this function.
	 */
    public double getRate()
    {
        return 0;
    }

	/**
	 * Resets the rate of the sensor to zero and clears any accumulators/counters to
	 * zero.
	 */
    public void reset()
    {
        ahrs.reset();
    }

	/**
	 *
	 * @return the current Position of the sensor in nominal units of the sensor.
	 *         Remember when using this method with a PID controller to ensure that
	 *         the setPoints of the PID controller are in the same unit as returned
	 *         by this function.
	 */
    public double getPos()
    {
        return ahrs.getPitch();
    }
    
}
