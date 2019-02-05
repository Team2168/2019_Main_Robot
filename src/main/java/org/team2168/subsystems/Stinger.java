package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;


public class Stinger extends Subsystem 
{
    VictorSP stingMotor1;
	VictorSP stingMotor2;
	AnalogPotentiometer stingPot;
    DigitalInput stingerHall1; // hall effects !!
    DigitalInput stingerHall2; // 

    private Stinger()
    {
        stingMotor1 = new VictorSP(RobotMap.STINGER_MOTOR_1);
        stingMotor2 = new VictorSP(RobotMap.STINGER_MOTOR_2);
		stingerHall1 = new DigitalInput(RobotMap.STINGER_HALL_1);
        stingerHall2 = new DigitalInput(RobotMap.STINGER_HALL_2);
        stingPot = new AnalogPotentiometer(RobotMap.STING_POT_VOLTAGE);
    }

    private void driveStingerDown(double speed)
    {
        if(stingerHall1.equals(1) && stingerHall2.equals(0))
            stingerMotor1.set(speed);
            stingerMotor2.set(speed);
    }

    private void driveStingerUp(double speed)
    {
        if(stingerHall1.equals(0) && stingerHall2.equals(1))
        stingerMotor1.set(-speed);
        stingerMotor2.set(-speed);
    }

    private double getRawPot()
    {
        return stingPot.;
    }


    public void initDefaultCommand() {
        
    }
}