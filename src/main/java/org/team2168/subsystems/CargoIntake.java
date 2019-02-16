package org.team2168.subsystems;
import org.team2168.Commands.DriveCargoIntakeWithJoystick;
import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CargoIntake extends Subsystem {
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
    /**
     * Positive value moves cargo inwards?
     */
    private SpeedController _drive;
    /**
     * negative value moves cargo outwards?
     */

    /**
     * the sharp IR Sensor will detect the presence of the cargo or measure the distance from the sensor to the cargo in volts
     */
    private AnalogInput _sharpIRSensor;

    
    
	public CargoIntake() {
        _drive = new VictorSP(RobotMap.CARGO_INTAKE_MOTOR);
        _sharpIRSensor = new AnalogInput(RobotMap.CARGO_INTAKE_SHARP_IR_SENSOR);
    }
    

    //this method will drive the motor for the cargo intake(causes wheels to move) Since its a double speed then
    //depending on what double you put in the paramaters then it can vary from moving balls out or moving balls in 
    //(positive moves ball out, negative moves ball in)
    public void drive(double speed)
    {
        _drive.set(speed);
    }


    //drivecargo is probably useless because why would you want to move
    //cargo both inwards and outwards, but just incase we will write it

    public double getRawIRVoltage()
    {
        return _sharpIRSensor.getVoltage();
    }

    public boolean isCargoPresent()//presence because in the readme it says that it will sense the "presence" of the cargo ;)
    {
        return (getRawIRVoltage() >= RobotMap.CARGO_IR_SENSOR_THRESHOLD);
    }
    //make method for sharp ir sensor which senses distance to cargo, 
    //use volts from the lift getrawpos etc.

        

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCargoIntakeWithJoystick());
	}
}
