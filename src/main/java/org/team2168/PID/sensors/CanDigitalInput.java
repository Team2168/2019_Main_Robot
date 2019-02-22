package org.team2168.PID.sensors;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CanDigitalInput{
    TalonSRX motor;

    public CanDigitalInput(TalonSRX motor){
        this.motor = motor;
    }

    public boolean getForwardLimit(){
        return this.motor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimit(){
        return this.motor.getSensorCollection().isRevLimitSwitchClosed();
    }
}