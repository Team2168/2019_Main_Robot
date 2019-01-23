package org.team2168.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Class to encapsulate all F310 functionality. No need to have multiple copies
 * in OI all the time
 *
 * @author kevin
 */
public class TILaunchPad extends Joystick {
	// Gamepad axis ports
	public static final int ANALOG_1 = 1;
	public static final int ANALOG_2 = 2;
	public static final int ANALOG_3 = 3;
	public static final int ANALOG_4 = 4;
	public static final int ANALOG_5 = 5;
	public static final int ANALOG_6 = 6;
	public static final int ANALOG_7 = 7;
	public static final int ANALOG_8 = 8;

	// Gamepad axis ports
	public static final int DIGITAL_1 = 1;
	public static final int DIGITAL_2 = 2;
	public static final int DIGITAL_3 = 3;
	public static final int DIGITAL_4 = 4;
	public static final int DIGITAL_5 = 5;
	public static final int DIGITAL_6 = 6;
	public static final int DIGITAL_7 = 7;
	public static final int DIGITAL_8 = 8;

	// Gamepad buttonsopp[
	public static final int BUTTON_1 = 1;
	public static final int BUTTON_2 = 2;
	public static final int BUTTON_3 = 3;
	public static final int BUTTON_4 = 4;
	public static final int BUTTON_5 = 5;
	public static final int BUTTON_6 = 6;
	public static final int BUTTON_7 = 7;
	public static final int BUTTON_8 = 8;
	public static final int BUTTON_9 = 9;
	public static final int BUTTON_10 = 10;
	public static final int BUTTON_11 = 11;
	public static final int BUTTON_12 = 12;
	public static final int BUTTON_13 = 13;
	public static final int BUTTON_14 = 14;
	public static final int BUTTON_15 = 15;
	public static final int BUTTON_16 = 16;

	/**
	 * Default constructor
	 * 
	 * @param port
	 *            the port the joystick is plugged into on the DS.
	 */
	public TILaunchPad(int port) {
		super(port);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel1() {
		return getRawAxis(ANALOG_1);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel2() {
		return getRawAxis(ANALOG_2);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel3() {
		return getRawAxis(ANALOG_3);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel4() {
		return getRawAxis(ANALOG_4);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel5() {
		return getRawAxis(ANALOG_5);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel6() {
		return getRawAxis(ANALOG_6);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel7() {
		return getRawAxis(ANALOG_7);
	}

	/**
	 * 
	 * @return double representing the raw axis value from -1 to 1
	 */
	public double getAnalogRaw_Channel8() {
		return getRawAxis(ANALOG_8);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton1() {
		return getRawButton(BUTTON_1);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton2() {
		return getRawButton(BUTTON_2);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton3() {
		return getRawButton(BUTTON_3);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton4() {
		return getRawButton(BUTTON_4);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton5() {
		return getRawButton(BUTTON_5);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton6() {
		return getRawButton(BUTTON_6);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton7() {
		return getRawButton(BUTTON_7);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton8() {
		return getRawButton(BUTTON_8);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton9() {
		return getRawButton(BUTTON_9);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton10() {
		return getRawButton(BUTTON_10);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton11() {
		return getRawButton(BUTTON_11);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton12() {
		return getRawButton(BUTTON_12);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton13() {
		return getRawButton(BUTTON_13);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton14() {
		return getRawButton(BUTTON_14);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton15() {
		return getRawButton(BUTTON_15);
	}

	/**
	 * Checks whether Button is being pressed
	 * 
	 * @return true if pressed
	 */
	public boolean isPressedButton16() {
		return getRawButton(BUTTON_16);
	}

	/**
	 * Returns an object of Button 1.
	 */
	public JoystickButton Button1() {
		return new JoystickButton(this, BUTTON_1);
	}

	/**
	 * Returns an object of Button 2.
	 */
	public JoystickButton Button2() {
		return new JoystickButton(this, BUTTON_2);
	}

	/**
	 * Returns an object of Button 3.
	 */
	public JoystickButton Button3() {
		return new JoystickButton(this, BUTTON_3);
	}

	/**
	 * Returns an object of Button 4.
	 */
	public JoystickButton Button4() {
		return new JoystickButton(this, BUTTON_4);
	}

	/**
	 * Returns an object of Button 5.
	 */
	public JoystickButton Button5() {
		return new JoystickButton(this, BUTTON_5);
	}

	/**
	 * Returns an object of Button 6.
	 */
	public JoystickButton Button6() {
		return new JoystickButton(this, BUTTON_6);
	}

	/**
	 * Returns an object of Button 7.
	 */
	public JoystickButton Button7() {
		return new JoystickButton(this, BUTTON_7);
	}

	/**
	 * Returns an object of Button 8.
	 */
	public JoystickButton Button8() {
		return new JoystickButton(this, BUTTON_8);
	}

	/**
	 * Returns an object of Button 9.
	 */
	public JoystickButton Button9() {
		return new JoystickButton(this, BUTTON_9);
	}

	/**
	 * Returns an object of Button 10.
	 */
	public JoystickButton Button10() {
		return new JoystickButton(this, BUTTON_10);
	}

	/**
	 * Returns an object of Button 11.
	 */
	public JoystickButton Button11() {
		return new JoystickButton(this, BUTTON_11);
	}

	/**
	 * Returns an object of Button 12.
	 */
	public JoystickButton Button12() {
		return new JoystickButton(this, BUTTON_12);
	}

	/**
	 * Returns an object of Button 13.
	 */
	public JoystickButton Button13() {
		return new JoystickButton(this, BUTTON_13);
	}

	/**
	 * Returns an object of Button 14.
	 */
	public JoystickButton Button14() {
		return new JoystickButton(this, BUTTON_14);
	}

	/**
	 * Returns an object of Button 15.
	 */
	public JoystickButton Button15() {
		return new JoystickButton(this, BUTTON_15);
	}

	/**
	 * Returns an object of Button 16.
	 */
	public JoystickButton Button16() {
		return new JoystickButton(this, BUTTON_16);
	}

	/**
	 * Returns an Button Object of Analog Channel 1. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog1() {
		return new JoystickAnalogButton(this, ANALOG_1, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 2. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog2() {
		return new JoystickAnalogButton(this, ANALOG_2, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 3. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog3() {
		return new JoystickAnalogButton(this, ANALOG_3, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 4. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog4() {
		return new JoystickAnalogButton(this, ANALOG_4, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 5. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog5() {
		return new JoystickAnalogButton(this, ANALOG_5, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 6. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog6() {
		return new JoystickAnalogButton(this, ANALOG_6, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 7. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog7() {
		return new JoystickAnalogButton(this, ANALOG_7, 0.5);
	}

	/**
	 * Returns an Button Object of Analog Channel 8. Enables use Analog Channel as
	 * button
	 */
	public JoystickAnalogButton ButtonAnalog8() {
		return new JoystickAnalogButton(this, ANALOG_8, 0.5);
	}

	public void setDigitalOutput1(boolean value) {
		this.setOutput(DIGITAL_1, value);
	}

	public void setDigitalOutput2(boolean value) {
		this.setOutput(DIGITAL_2, value);
	}

	public void setDigitalOutput3(boolean value) {
		this.setOutput(DIGITAL_3, value);
	}

	public void setDigitalOutput4(boolean value) {
		this.setOutput(DIGITAL_4, value);
	}

	public void setDigitalOutput5(boolean value) {
		this.setOutput(DIGITAL_5, value);
	}

	public void setDigitalOutput6(boolean value) {
		this.setOutput(DIGITAL_6, value);
	}

	public void setDigitalOutput7(boolean value) {
		this.setOutput(DIGITAL_7, value);
	}

}
