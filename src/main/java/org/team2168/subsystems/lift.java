/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;
import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * 2x VictorSP motor controllers to drive lift up/down ..- Positive values move upwards / Negative values move down ..- On 30A fuses on the PDP
2x hall effect sensors (discrete inputs) for fully raised & fully lowered position indications
1x pneumatic Double Solenoid for brake ..- KForward is high speed / KReverse is low speed
1x Encoder/10 turn potentiometer for lift position 
hall effect sensors are used in tandem with the pot. to stop the lift from going up so it doesn't break the drivetrain
 */
public class lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  /**
   * a motor to raise/lower the lift (1 of 2)
   */
  static VictorSP liftMotor1;
  /**
   * a motor to raise/lower the lift (2 of 2)
   */
  static VictorSP liftMotor2;
  /**
   * a double solenoid with a rubber pad on the end to stop a gear in the lift gearbox to "lock" the lift at the specified position
   */
  public static DoubleSolenoid liftDSolenoid;
  /**
   * a hall effect sensor to work in tandem with the potentiometer to ensure that the lift stays within the minimum height
   */
  private DigitalInput liftFullyDown;
  /**
   * a hall effect sensor to work in tandem with the potentiometer to ensure that the lift stays within the maximum height
   */
  private DigitalInput liftFullyUp;
  /**
   * tracks the position of the lift
   */
  private AnalogPotentiometer liftPosition;
  /**
   * a variable which surns true if either a hall effect sensor or potentiometer reports true or at the maximum height
   */
  private boolean _liftImmovable;
 
  private boolean _solenoidPos;

  private double _liftMove;
  //default constructors
  private lift(){
    liftMotor1=new VictorSP(RobotMap.LIFT_MOTOR_1);
    liftMotor2=new VictorSP(RobotMap.LIFT_MOTOR_2);
    liftDSolenoid=new DoubleSolenoid(RobotMap.LIFT_BRAKE_ENGAGE_PCM, RobotMap.LIFT_BRAKE_DISENGAGE_PCM);
    liftFullyDown = new DigitalInput(RobotMap.LIFT_FULLY_DOWN_LIMIT);
    liftFullyUp=new DigitalInput(RobotMap.LIFT_FULLY_UP_LIMIT);
    liftPosition=new AnalogPotentiometer(RobotMap.LIFT_POSITION_POT);
    _liftMove=0.0;
    _liftImmovable=false;
    _solenoidPos=false;

  }


private void driveLift (double _liftSpeed){

liftMotor1.set(_liftSpeed);
liftMotor2.set(_liftSpeed);

}

public void _getLiftPosition() {
  if (liftFullyDown.get()==true||liftFullyUp.get()==true||liftPosition.get()>=RobotMap.LIFT_POT_VOLTAGE_MAX||liftPosition.get()>=RobotMap.LIFT_POT_0_HEIGHT_INCHES|| liftDSolenoid.get() == Value.kForward) {
    _liftImmovable=true;
  }
  else{
    _liftImmovable=false;
  }
}


  public void driveLift(){
    _getLiftPosition();
    if (_solenoidPos==false){
      if (_liftImmovable=false){
        driveLift();
      }
      else{
        if (_liftMove!=Math.abs(_liftMove)){
          driveLift();
        }
      }
    }
    else{
      liftDSolenoid.set(Value.kReverse);
    }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
