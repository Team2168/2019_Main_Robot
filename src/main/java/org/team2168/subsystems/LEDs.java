/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LEDs extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static I2C _i2c;
  private static LEDs _instance;

  private LEDs()
  {
    _i2c = new I2C(RobotMap.I2C_PORT, RobotMap.I2C_ADDRESS);

  }

  /**
   * Calls instance object and makes it a singleton object of type LEDs
   */
  public static LEDs getInstance()
  {
    if (_instance == null)
    {
      _instance = new LEDs();
    }
    return _instance;
  }

  /**
   * Sends the indicated pattern number to the LEDs Arduino
   */
  public void writePattern(int pattern)
  {
    _i2c.write(RobotMap.I2C_ADDRESS, pattern);
  }

  /**
   * Sends the indicated pattern and HSV color values to the Arduino HSV
   * @param hue A color hue between 0 and 255 as used by the FastLED HSV colors
   * @param sat the saturation of the color between 0 and 255
   * @param val the brightness of the color between 0 and 255
   */
  public void writePatternOneColor(int pattern, int hue, int sat, int val)
  {
    byte lightByte[] = new byte[4];
    lightByte[0] = (byte) hue;
    lightByte[1] = (byte) sat;
    lightByte[2] = (byte) val;
    lightByte[3] = (byte) pattern;
    _i2c.writeBulk(lightByte);
  }

  public void writePatternTwoColors(int pattern, int hue1, int sat1, int val1, int hue2, int sat2, int val2)
  {
    byte lightByte[] = new byte[7];
    lightByte[0] = (byte) hue2;
    lightByte[1] = (byte) sat2;
    lightByte[2] = (byte) val2;
    lightByte[3] = (byte) hue1;
    lightByte[4] = (byte) sat1;
    lightByte[5] = (byte) val1;
    lightByte[6] = (byte) pattern;
    _i2c.writeBulk(lightByte);

  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
