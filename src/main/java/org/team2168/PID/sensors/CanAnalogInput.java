package org.team2168.PID.sensors;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CanAnalogInput
{
    TalonSRX motor;

    public static final double kSCALE_5_VOLTS = 5.0;
    public static final double kSCALE_3_3_VOLTS = 3.3;

    private static final double MAX_ADC_VALUE = 1023.0; //From CTRE documentation of SRX motors

    private double scale;

    /**
     * Default construct takes in a reference to a talon SRX that has already been instantiated. Assumes the voltage range of the sensor is 0 to 5 volts. 
     * @param motor
     */
    public CanAnalogInput(TalonSRX motor){
        this.motor = motor;
        this.scale = this.kSCALE_5_VOLTS;
    }

    public CanAnalogInput(TalonSRX motor, double scale)
    {
        this(motor);
        this.scale = scale;
    }

    /**
     * SRX doesn't return voltage it returns the ADC scaled from 0 to 1023. This returns the raw ADC
     * @return
     */
    public int getRAW_ADC()
    {
        return this.motor.getSensorCollection().getAnalogInRaw();
    }

    /**
     * 
     * @return 0 to 5 volts on analog pin of sensor
     */
	public double getVoltage()
	{
		return this.motor.getSensorCollection().getAnalogInRaw() * (this.scale / CanAnalogInput.MAX_ADC_VALUE);
	}
}