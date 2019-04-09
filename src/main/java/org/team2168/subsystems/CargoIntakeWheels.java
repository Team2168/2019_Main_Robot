package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.sensors.CanAnalogInput;
import org.team2168.commands.LEDs.WheelsInPattern;
import org.team2168.commands.LEDs.WheelsOutPattern;
import org.team2168.commands.cargoIntake.DriveCargoIntakeWithJoystick;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * negative value moves cargo outwards?
 */

/**
 * the sharp IR Sensor will detect the presence of the cargo or measure the distance from the sensor to the cargo in volts
 */
public class CargoIntakeWheels extends Subsystem {

    public TalonSRX _intakeMotor;
    private CanAnalogInput _sharpIRSensor;
    public static volatile double _driveVoltage;
    private static CargoIntakeWheels _instance;
    public double cargoIntakeWheelsSpeedForLEDs;


	private CargoIntakeWheels() {
        _intakeMotor = new TalonSRX(RobotMap.CARGO_INTAKE_MOTOR_PDP);
        //_intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        _sharpIRSensor = new CanAnalogInput(_intakeMotor, CanAnalogInput.kSCALE_3_3_VOLTS);

        ConsolePrinter.putNumber("Cargo Raw IR", () -> {return getRawIRVoltage();}, true, false);
        ConsolePrinter.putBoolean("isCargoPresent", () -> {return isCargoPresent();}, true, false);
        ConsolePrinter.putNumber("Intake motor voltage", () -> {return _driveVoltage;}, true, false);

    }

    public static CargoIntakeWheels getInstance(){
        if(_instance == null) {
            _instance = new CargoIntakeWheels();
        }
        return _instance;
    }
    

    // this method will drive the motor for the cargo intake(causes wheels to move)
    // Since its a double speed then
    // depending on what double you put in the paramaters then it can vary from
    // moving balls out or moving balls in
    // (positive moves ball out, negative moves ball in)
    public void drive(double speed)
    {
        this.cargoIntakeWheelsSpeedForLEDs = speed;
        if (RobotMap.CARGO_INTAKE_MOTOR_REVERSE)
            speed = -speed;

        _intakeMotor.set(ControlMode.PercentOutput,speed);
        _driveVoltage = Robot.pdp.getBatteryVoltage() * speed;
        
    }

    // drivecargo is probably useless because why would you want to move
    // cargo both inwards and outwards, but just incase we will write it

    public double getRawIRVoltage()
    {
        return _sharpIRSensor.getVoltage();
    }

    public boolean isCargoPresent()
    {
        if (Robot.isPracticeRobot())
            return (getRawIRVoltage() >= RobotMap.CARGO_INTAKE_IR_THRESHOLD_MAX_PBOT);
        else
            return (getRawIRVoltage() >= RobotMap.CARGO_INTAKE_IR_THRESHOLD_MIN && getRawIRVoltage() <= RobotMap.CARGO_INTAKE_IR_THRESHOLD_MAX);
    }
    // make method for sharp ir sensor which senses distance to cargo,
    // use volts from the lift getrawpos etc.
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveCargoIntakeWithJoystick());
    }
}
