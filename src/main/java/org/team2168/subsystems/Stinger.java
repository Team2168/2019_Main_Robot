package org.team2168.subsystems;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Stinger extends Subsystem
{
    AveragePotentiometer _stingerPotLeft;
    AveragePotentiometer _stingerPotRight;

    private static DoubleSolenoid _ratchet;

    private static Stinger _instance;

    private Stinger()
    {
      
        _ratchet = new DoubleSolenoid(RobotMap.PCM_CAN_ID_BELLYPAN, RobotMap.STINGER_RACHET_ENGAGE_PCM,
                RobotMap.STINGER_RACHET_DISENGAGE_PCM);

        if (Robot.isPracticeRobot())
        {
            _stingerPotLeft = new AveragePotentiometer(RobotMap.STINGER_AVERAGE_POTENTIOMETER_LEFT,
                    RobotMap.STINGER_LEFT_POT_VOLTAGE_0_PBOT, RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES_PBOT,
                    RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX_PBOT, RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES_PBOT,
                    RobotMap.STINGER_AVG_ENCODER_VAL);

            _stingerPotRight = new AveragePotentiometer(RobotMap.STINGER_AVERAGE_POTENTIOMETER_RIGHT,
                    RobotMap.STINGER_RIGHT_POT_VOLTAGE_0_PBOT, RobotMap.STINGER_RIGHT_POT_0_HEIGHT_INCHES_PBOT,
                    RobotMap.STINGER_RIGHT_POT_VOLTAGE_MAX_PBOT, RobotMap.STINGER_RIGHT_POT_MAX_HEIGHT_INCHES_PBOT,
                    RobotMap.STINGER_AVG_ENCODER_VAL);
        }
        else
        {
            _stingerPotLeft = new AveragePotentiometer(RobotMap.STINGER_AVERAGE_POTENTIOMETER_LEFT,
                    RobotMap.STINGER_LEFT_POT_VOLTAGE_0, RobotMap.STINGER_LEFT_POT_0_HEIGHT_INCHES,
                    RobotMap.STINGER_LEFT_POT_VOLTAGE_MAX, RobotMap.STINGER_LEFT_POT_MAX_HEIGHT_INCHES,
                    RobotMap.STINGER_AVG_ENCODER_VAL);

            _stingerPotRight = new AveragePotentiometer(RobotMap.STINGER_AVERAGE_POTENTIOMETER_RIGHT,
                    RobotMap.STINGER_RIGHT_POT_VOLTAGE_0, RobotMap.STINGER_RIGHT_POT_0_HEIGHT_INCHES,
                    RobotMap.STINGER_RIGHT_POT_VOLTAGE_MAX, RobotMap.STINGER_RIGHT_POT_MAX_HEIGHT_INCHES,
                    RobotMap.STINGER_AVG_ENCODER_VAL);

        }

        ConsolePrinter.putBoolean("Stinger Ratchet Enagaged", () -> {return Robot.stinger.isRatchetEngaged();}, true, false);
        ConsolePrinter.putBoolean("Stinger Ratchet Disengaged", () -> {return Robot.stinger.isRatchetDisengaged();}, true, false);

        ConsolePrinter.putNumber("Left Stinger Pot Raw", () -> {return Robot.stinger.getLeftPotRaw();}, true, false);
        ConsolePrinter.putNumber("Right Stinger Pot Raw", () -> {return Robot.stinger.getRightPotRaw();}, true, false);

        ConsolePrinter.putNumber("Left Stinger Pot Inch", () -> {return Robot.stinger.getLeftPotPos();}, true, false);
        ConsolePrinter.putNumber("Right Stinger Pot Inch", () -> {return Robot.stinger.getRightPotPos();}, true, false);
    }

    /**
     * Singleton constructor of the plunger arm pivot
     * 
     */
    public static Stinger getInstance()
    {
        if (_instance == null)
            _instance = new Stinger();
        return _instance;
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

    /**
     * Engages the stinger ratchet to lock stingers
     */
    public void engageRatchet()
    {
        _ratchet.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Disengages the stinger ratchet to allow the stingers to retract or extend
     */
    public void disengageRatchet()
    {
        _ratchet.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * 
     * @return true if last commanded shift was to engageRatchet
     */
    public boolean isRatchetEngaged()
    {
        return _ratchet.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * 
     * @return true if last commanded shift was to disengageRatchet
     */
    public boolean isRatchetDisengaged()
    {
        return _ratchet.get() == DoubleSolenoid.Value.kReverse;
    }

    public void initDefaultCommand()
    {
    }
}