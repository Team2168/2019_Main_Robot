package org.team2168.subsystem;

import org.team2168.Commands.DriveCargoIntakeWITHJoystick;
import org.team2168.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private SpeedController cargoMotor1;
    /**
     * negative value moves cargo outwards?
     */

    /**
     * the sharp IR Sensor will detect the presence of the cargo or measure the distance from the sensor to the cargo in volts
     */
    private AnalogInput sharpIRSensor;
    /**
     * the double solenoid will extend and retract a piston to punch the ball/cargo out, koff = intake open, kon = intake closed
     */
    private DoubleSolenoid punchCargoSolenoid;
    
    
	public CargoIntake() {
        cargoMotor1 = new VictorSP(RobotMap.CARGO_INTAKE_MOTOR_1);
        punchCargoSolenoid = new DoubleSolenoid(RobotMap.CARGO_PUNCH_BALL_EXTENDED_PCM, RobotMap.CARGO_PUNCH_BALL_RETRACTED_PCM);
        sharpIRSensor = new AnalogInput(RobotMap.CARGO_INTAKE_SHARP_IR_SENSOR);
    }
    
    public void driveCargoMotor1(double speed)
    {
        cargoMotor1.set(speed);
    }


    //drivecargo is probably useless because why would you want to move
    //cargo both inwards and outwards, but just incase we will write it

    public void intakeOpen()//or i should change it to extend
    {
        punchCargoSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void intakeClose()//or i should change it to retract?
    {
        punchCargoSolenoid.set(DoubleSolenoid.Value.kReverse);//should be kon but isnt working
    }

    public boolean cargoExtended(){
        return punchCargoSolenoid.get() == Value.kForward; //inspiration from allysas subsystem
    }

    public boolean cargoRetracted(){
        return punchCargoSolenoid.get() == Value.kReverse; //inspiration from allysas subsystem
    }
    public double getRawIRVoltage()
    {
        return sharpIRSensor.getVoltage();
    }

    public boolean cargoPresence()//presence because in the readme it says that it will sense the "presence" of the cargo ;)
    {
        return (getRawIRVoltage() >= RobotMap.CARGO_INTAKE_SHARP_IR_SENSOR);
    }
    //make method for sharp ir sensor which senses distance to cargo, 
    //use volts from the lift getrawpos etc.

        

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCargoIntakeWITHJoystick());
	}
}
