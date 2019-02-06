package org.team2168.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import org.team2168.PID.sensors.AveragePotentiometer;
import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;


public class Stinger extends Subsystem 
{
    AveragePotentiometer stingPotLeft;
    AveragePotentiometer stingPotRight;
    DigitalInput stingerHallLeft; //
    DigitalInput stingerHallRight; // 

    private Stinger()
    {
		stingerHallLeft = new DigitalInput(RobotMap.STINGER_HALL_1);
        stingerHallRight = new DigitalInput(RobotMap.STINGER_HALL_2);
        stingPotLeft = new AveragePotentiometer(RobotMap.STING_POT_VOLTAGE);
        stingPotRight = new AveragePotentiometer(RobotMap.STING_POT_VOLTAGE);
    }

    public void driveStingerDown(double speed)
    {
        if(stingerHallLeft == true && stingerHallRight == true)
        {    
        }
    }

    public void driveStingerUp(double speed)
    {
        if(stingerHallLeft == false && stingerHallRight == false)
       {
       }
    }

    public static void initDefaultCommand() {
        
    }
}