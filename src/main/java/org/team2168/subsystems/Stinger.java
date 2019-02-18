package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Stinger extends Subsystem {
    AveragePotentiometer _stingerPotLeft;
    AveragePotentiometer _stingerPotRight;
    DigitalInput _stingerRightRatchetEngagedHallEffect;
    DigitalInput _stingerLeftRatchetEngagedHallEffect; 

    private static Stinger _instance;
    private Stinger()
    {
		_stingerLeftRatchetEngagedHallEffect  = new DigitalInput(RobotMap.STINGER_LEFT_RATCHET_ENGAGED);
        _stingerRightRatchetEngagedHallEffect = new DigitalInput(RobotMap.STINGER_RIGHT_RATCHET_ENGAGED);

        if (Robot.isPracticeRobot()) {
            _stingerPotLeft = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_LEFT,
                RobotMap.STINGER_LEFT_POT_VOLTAGE_0_PBOT,
                RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES_PBOT, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX_PBOT,
                RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES_PBOT, 
                RobotMap.STINGER_AVG_ENCODER_VAL);

            _stingerPotRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_0_PBOT,
                RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES_PBOT, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX_PBOT,
                RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES_PBOT, 
                RobotMap.STINGER_AVG_ENCODER_VAL);
        } 
        else 
        {
            _stingerPotLeft = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_LEFT,
                RobotMap.STINGER_LEFT_POT_VOLTAGE_0,
                RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX,
                RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES, 
                RobotMap.STINGER_AVG_ENCODER_VAL);

            _stingerPotRight = new AveragePotentiometer(RobotMap.MONKEY_BAR_AVERAGE_POTENTIOMETER_RIGHT, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_0,
                RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES, 
                RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX,
                RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES, 
                RobotMap.STINGER_AVG_ENCODER_VAL);

        }

    }

    /**
     * Singleton constructor of the plunger arm pivot
    * 
    */
    public static Stinger getInstance() {
        if (_instance == null)
        _instance = new Stinger();
        return _instance;
    }

    public boolean isLeftStingerRatchetEngaged()
    {
        return _stingerLeftRatchetEngagedHallEffect.get();
    }

    public boolean isRightStingerRatchetEngaged()
    {
        return _stingerRightRatchetEngagedHallEffect.get();
    }

    public double getLeftPotRaw()
    {
        return _stingerPotLeft.getRawPos();
    }

    public double getRightPotRaw()
    {
        return _stingerPotRight.getRawPos();
    }

    public double getLeftPotPos()
    {
        return _stingerPotLeft.getPos();
    }

    public double getRightPotPos()
    {
        return _stingerPotRight.getPos();
    }

    public void initDefaultCommand() { 
    }
}