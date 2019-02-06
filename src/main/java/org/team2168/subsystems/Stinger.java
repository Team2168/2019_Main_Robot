package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;


public class Stinger extends Subsystem 
{
    AveragePotentiometer _stingPotLeft;
    AveragePotentiometer _stingPotRight;
    DigitalInput _stingerHallLeft; //
    DigitalInput _stingerHallRight; // 

    private Stinger()
    {
		_stingerHallLeft = new DigitalInput(RobotMap.STINGER_HALL_1);
        _stingerHallRight = new DigitalInput(RobotMap.STINGER_HALL_2);
        _stingPotLeft = new AveragePotentiometer(RobotMap.STING_POT_VOLTAGE);
        _stingPotRight = new AveragePotentiometer(RobotMap.STING_POT_VOLTAGE);
    }

    public boolean isLeftStingDown()
    {
        return _stingerHallLeft.get();
    }

    public boolean isRightStingDown()
    {
        return _stingerHallRight.get();
    }

    public double getRawLeftPot()
    {
        return _stingPotLeft.getRawPos();
    }

    public double getRawRightPot()
    {
        return _stingPotRight.getRawPos();
    }

    public static void initDefaultCommand() { 
    }
}