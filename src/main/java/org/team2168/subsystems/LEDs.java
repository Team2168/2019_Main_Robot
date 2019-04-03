/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import java.util.TimerTask;

import org.team2168.RobotMap;
import org.team2168.commands.LEDs.TeleopWithoutGamePiecePattern;

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

  private java.util.Timer executor;
  private static final long THREAD_PERIOD = 20; // ms - max poll rate on sensor.

  private boolean writePattern;
  private boolean writePatternOneColor;
  private boolean writePatternTwoColors;

  int pattern;
  byte lightByteOneColor[] = new byte[4];
  byte lightByteTwoColor[] = new byte[7];



  private LEDs()
  {
    _i2c = new I2C(RobotMap.I2C_PORT, RobotMap.I2C_ADDRESS);
    startThread();
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
  
  public void startThread()
  {
    executor = new java.util.Timer();
    executor.schedule(new LedsUpdateTask(this), 0L, THREAD_PERIOD);
  }

  /**
   * Sends the indicated pattern number to the LEDs Arduino
   */
  public void writePattern(int pattern)
  {
    this.pattern = pattern;
    writePattern = true;
    //_i2c.write(RobotMap.I2C_ADDRESS, pattern);
  }

  /**
   * Sends the indicated pattern and HSV color values to the Arduino HSV
   * @param hue A color hue between 0 and 255 as used by the FastLED HSV colors
   * @param sat the saturation of the color between 0 and 255
   * @param val the brightness of the color between 0 and 255
   */
  public void writePatternOneColor(int pattern, int hue, int sat, int val)
  {
    lightByteOneColor[0] = (byte) hue;
    lightByteOneColor[1] = (byte) sat;
    lightByteOneColor[2] = (byte) val;
    lightByteOneColor[3] = (byte) pattern;
    writePatternOneColor = true;
    //_i2c.writeBulk(lightByteOneColor);
  }

  public void writePatternTwoColors(int pattern, int hue1, int sat1, int val1, int hue2, int sat2, int val2)
  {
    lightByteTwoColor[0] = (byte) hue2;
    lightByteTwoColor[1] = (byte) sat2;
    lightByteTwoColor[2] = (byte) val2;
    lightByteTwoColor[3] = (byte) hue1;
    lightByteTwoColor[4] = (byte) sat1;
    lightByteTwoColor[5] = (byte) val1;
    lightByteTwoColor[6] = (byte) pattern;
    writePatternTwoColors = true;
    //_i2c.writeBulk(lightByteTwoColor);

  }

  private void run()
  {
    if(writePattern)
    {
      _i2c.write(RobotMap.I2C_ADDRESS, pattern);
      writePattern = false;
    }
    else if(writePatternOneColor)
    {
      _i2c.writeBulk(lightByteOneColor);
      writePatternOneColor = false;
    }
    else if(writePatternTwoColors)
    {
      _i2c.writeBulk(lightByteTwoColor);
      writePatternTwoColors = false;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopWithoutGamePiecePattern());
  }


  private class LedsUpdateTask extends TimerTask {
    private LEDs leds;

    private LedsUpdateTask(LEDs leds) {
      if (leds == null) {
        throw new NullPointerException("LEDs pointer null");
      }
      this.leds = leds;
    }

    /**
     * Called periodically in its own thread
     */
    public void run() {
      leds.run();
 
    }
  }
}
