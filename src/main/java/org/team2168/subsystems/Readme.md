 # Subsystems
## Actuator summary per subsystem
| Subsystem                      | Motors | Pneumatics | Sensors |
|--------------------------------|--------|------------|---------|
| Lift                           | 2      | 1          | 3       |
| Claw                           | 2      | 1          | 1       |
| Harpoon                        | 0      | 2          | 1       |
| Monkey Bar                     | 4      | 1          | 2       |
| Drivetrain                     | 6      | 2          | 4       |
| Stingers                       | 0      | 1          | 2       |
| Hatch Panel Floor Intake (HPI) | 1      | 1          | 1       |
| **Total**                      | **15** | **9**      | **14**  |

## Robot Map

| Pin | PDP | Pin | DIO | Pin | Analog | PIN | CAN_ID | Pin | PCM1 | Pin | PCM2 | SPI | I2C |
|-----|-----|-----|-----|-----|--------|-----|--------|-----|------|-----|-----|------|-----|
| NA  | 40 AMP | 0 | RDT A Encoder | 0 |  | 0 | RDT 1 | 0 | DT Disable | 0 |  | NAVX IMU |
| 0 | RDT 1 | 1 | RDT B Encoder | 1 |  | 1 | RDT 2 | 1 | Stinger PTO Engage | 1 |  | LED Lights |
| 1 | RDT 2 | 2 | LDT A Encoder | 2 |  | 2 | RDT 3 | 2 | Hatch Intake Lower | 2 | Ball Punch Extend |  |
| 2 | RDT 3 | 3 | LDT B Encoder | 3 |  | 3 | LDT 1 | 3 | Hatch Intake Rise | 3 | Ball Punch Retract |  |
| 3 | LIFT 1 | 4 | Lift Raised | X | IR Sensor Cargo | 4 | LDT 2 | 4 |  | 4 | Harpoon Extend |  |
| 12 | LIFT 2 | 5 | Lift Lowered | X | IR Sensor Hatch | 5 | LDT 3 | 5 |  | 5 | Harpoon Retract |  |
| 13 | LDT 1 | 6 | Monkey Bar Raised | X | Stinger Pos R | 6 | LIFT 1 | 6 | Lift Brake Engage | 6 | Harpoon Engage |  |
| 14 | LDT 2 | 7 |  | X | Stinger Pos L | 7 | LIFT 2 | 7 | Lift Brake Disengage | 7 | Harpoon Disengage |  |
| 15 | LDT 3 | 8 |  | X | Lift Pos | 8 | Intake Rotate |  |  |  |  |  |
| NA | 30-20 AMP | 9 |  | X | Hatch Intake IR | 9 | Ball Intake |  |  |  |  |  |
| 4 | Intake Rotation Motor |  |  | X | Monkey Bar Pos | 10 | Monkey Bar Raise 1 |  |  |  |  |  |
| 5 | Ball Intake |  |  | X | Hab Sensor FWD | 11 | Monkey Bar Raise 2 |  |  |  |  |  |
| 6 | PENDING |  |  | X | Hab Sensor AFT | 12 | Monkey Bar Wheels 1 |  |  |  |  |  |
| 7 | Monkey Bar Raise 1 |  |  |  |  | 13 | Monkey Bar Wheels 2 |  |  |  |  |  |
| 8 | Monkey Bar Raise 2 |  |  |  |  | 14 |  |  |  |  |  |  |
| 9 | Monkey Bar Wheels 1 |  |  |  |  |  |  |  |  |  |  |  |
| 10 | Monkey Bar Wheels 2 |  |  |  |  |  |  |  |  |  |  |  |
| 11 | PENDING |  |  |  |  |  |  |  |  |  |  |  |

## Subsystem Requirements and Descriptions

### Monkey Bar (MB)
Serves dual function of climbing to 3rd level on endgame and intaking cargo during match. This system will need to work in tandem with the stingers in order to lift the robothorizontally onto the 3rd level.

| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| MB01 | private | VictorSP | _rightRaiseMotor | Range of -25 deg to 90 deg from vertical |
| MB02 | private | VictorSP | _leftRaiseMotor | Range of -25 deg to 90 deg from vertical |
| MB03 | private | VictorSP | _leftWheels | Active intake on wheels and drive robot onto third level |
| MB04 | private | VictorSP | _rightWheels | Active intake on wheels and drive robot onto third level |
| MB05 | private | DoubleSolenoid | _brake | Break to hold Monkey Bar arms in a fixed position |
| MB06 | private | DigitalInput | _isFullyRaised | Boolean sensor to indicate if MB is raised fully or not |
| MB07 | private | AverageEncoder | _currPosition | Rotary position sensor (Potentiometer) to indicate the current position of the MB |

| Pin | PDP | Pin | DIO | Pin | Analog | PIN | CAN_ID | Pin | PCM1 | Pin | PCM2 | SPI | I2C |
|-----|-----|-----|-----|-----|--------|-----|--------|-----|------|-----|-----|------|-----|
| NA  | 40 AMP | 0 | RDT A Encoder | 0 |  | 0 | RDT 1 | 0 | DT Disable | 0 |  | NAVX IMU |
| 0 | RDT 1 | 1 | RDT B Encoder | 1 |  | 1 | RDT 2 | 1 | Stinger PTO Engage | 1 |  | LED Lights |
| 1 | RDT 2 | 2 | LDT A Encoder | 2 |  | 2 | RDT 3 | 2 | Hatch Intake Lower | 2 | Ball Punch Extend |  |
| 2 | RDT 3 | 3 | LDT B Encoder | 3 |  | 3 | LDT 1 | 3 | Hatch Intake Rise | 3 | Ball Punch Retract |  |
| 3 | LIFT 1 | 4 | Lift Raised | X | IR Sensor Cargo | 4 | LDT 2 | 4 |  | 4 | Harpoon Extend |  |
| 12 | LIFT 2 | 5 | Lift Lowered | X | IR Sensor Hatch | 5 | LDT 3 | 5 |  | 5 | Harpoon Retract |  |
| 13 | LDT 1 | 6 | Monkey Bar Raised | X | Stinger Pos R | 6 | LIFT 1 | 6 | Lift Brake Engage | 6 | Harpoon Engage |  |
| 14 | LDT 2 | 7 |  | X | Stinger Pos L | 7 | LIFT 2 | 7 | Lift Brake Disengage | 7 | Harpoon Disengage |  |
| 15 | LDT 3 | 8 |  | X | Lift Pos | 8 | Intake Rotate |  |  |  |  |  |
| NA | 30-20 AMP | 9 |  | X | Hatch Intake IR | 9 | Ball Intake |  |  |  |  |  |
| 4 | Intake Rotation Motor |  |  | X | Monkey Bar Pos | 10 | Monkey Bar Raise 1 |  |  |  |  |  |
| 5 | Ball Intake |  |  | X | Hab Sensor FWD | 11 | Monkey Bar Raise 2 |  |  |  |  |  |
| 6 | PENDING |  |  | X | Hab Sensor AFT | 12 | Monkey Bar Wheels 1 |  |  |  |  |  |
| 7 | Monkey Bar Raise 1 |  |  |  |  | 13 | Monkey Bar Wheels 2 |  |  |  |  |  |
| 8 | Monkey Bar Raise 2 |  |  |  |  | 14 |  |  |  |  |  |  |
| 9 | Monkey Bar Wheels 1 |  |  |  |  |  |  |  |  |  |  |  |
| 10 | Monkey Bar Wheels 2 |  |  |  |  |  |  |  |  |  |  |  |
| 11 | PENDING |  |  |  |  |  |  |  |  |  |  |  |


| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| | public | VictorSP | liftMotor1 |a motor to raise/lower the lift (1 of 2) |
| | public | VictorSP| liftMotor2 | a motor to raise/lower the lift (2 of 2) |
| | private| DigitalInput | liftFullyDown | a hall effect sensor to work in tandem with the potentiometer to ensure that the lift stays within the minimum height |
| | private | DigitalInput | liftFullyUp | a hall effect sensor to work in tandem with the potentiometer to ensure that the lift stays within the maximum height
 |
| | private | AnalogPotientometer | liftPosition | tracks the position of the lift |

liftHardStop
| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| | public | DoubleSolenoid | liftDSolenoid | a double solenoid with a rubber pad on the end to stop a gear in the lift gearbox to "lock" the lift at the specified position |


#### Requirements: Interlocks, Functionality, or External
**Interlocks** represent scenarios where the robot may be capable of harming itself by its own subsystem action or by the action of another subsystem (e.g. Intake may not lift higher than the crossbar of the lift when it is rotated towards the back of the robot).<br>
**Functionality** is a capability the subsystem needs to have (e.g. The intake shall hold the cargo in place until it is commanded to shoot or drop the cargo).<br>
**External** represent scenarios when a subsystem will need to collaborate with or consider another subsystem for a particular action (e.g. Stingers shall lower at the same rate as MonkeyBars to horizontally lift the robot to the third stage).<br>

| Requirement Type | Subsystem | Component | Interfaces with | Description |
|------------------|-----------|-----------|-----------------|-------------|
| Interlock | Monkey Bar (MB) |  | Monkey Bar (MB) |  |
| Functionality | Monkey Bar (MB) |  | Monkey Bar (MB) | Must be raised at the beggining of the match |
| Warning | Monkey Bar (MB) |  | Monkey Bar (MB) | Consider raising when not actively intaking |
| External | Monkey Bar (MB) |  | Stingers | During end game, Stingers will need to raise the robot at the same rate as the Monkey Bar |

<img src="http://yuml.me/diagram/scruffy/class/[MonkeyBar|-VictorSP _rightRaiseMotor;-VictorSP _leftRaiseMotor;-VictorSP _rightWheelMotor;-VictorSP _leftWheelMotor;-DoubleSolenoid _brake;-DigitalInput _fullyRaised; -AverageEncoder _currPosition |+MonkeyBar();+methodA(String word)]" >


### Drivetrain (D)
Powers the robot's movement, controlling speed and direction of travel. The motors provided to the drivetrain are temporarily rerouted to the stingers during end game. 

| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| D01 | private | SpeedController | _leftMotor1 | motor controller for wheels on the left side of the chassis |
| D02 | private | SpeedController | _leftMotor2 | motor controller for wheels on the left side of the chassis |
| D03 | private | SpeedController | _leftMotor3 | motor controller for wheels onthe left side of the chassis |
| D04 | private | SpeedController | _rightMotor1 | motor controller for wheels on the right side of the chassis |
| D05 | private | SpeedController | _rightMotor2 | motor controller for wheels on the right side of the chassis |
| D06 | private | SpeedController | _rightMotor3 | motor controller for wheels on the right side of the chassis |
| D07 | private | ADXRS453 Gyro | _gyroSPI | rotational position sensor to determine current heading of the robot |
| D08 | private | AverageEncoder | _drivetrainLeftEncoder | linear position sensor that determines how far the robot has travelled |
| D09 | private | AverageEncoder | _drivetrainRightEncoder | linear position sensor that determines how far the robot has travelled |
| D10 | private | double | _leftMotor1Voltage | gives raw data of voltage sent to the motor controller |
| D11 | private | double | _leftMotor2Voltage | gives raw data of voltage sent to the motor controller |
| D12 | private | double | _leftMotor3Voltage | gives raw data of voltage sent to the motor controller |
| D13 | private | double | _rightMotor1Voltage | gives raw data of voltage sent to the motor controller | 
| D14 | private | double | _rightMotor2Voltage | gives raw data of voltage sent to the motor controller | 
| D15 | private | double | _rightMotor3Voltage | gives raw data of voltage sent to the motor controller |
| D16 | public | IMU | _imu | averages encoder values to find average distance travelled |
| D17 | public | PIDPosition | _drivetrainPosController | helps to control the robot's position (ie how far it drives), esp during autos. Uses PID |
| D18 | public | PIDPosition | _rotateDriveStraightController | helps to stabilize the robot's heading when using the Gun Style controller or during autos |
| D19 | private | Drivetrain | _instance | object designed to create a singleton object of the Drivetrain

#### Requirements: Interlocks, Functionality, or External
**Interlocks** represent scenarios where the robot may be capable of harming itself by its own subsystem action or by the action of another subsystem (e.g. Intake may not lift higher than the crossbar of the lift when it is rotated towards the back of the robot).<br>
**Functionality** is a capability the subsystem needs to have (e.g. The intake shall hold the cargo in place until it is commanded to shoot or drop the cargo).<br>
**External** represent scenarios when a subsystem will need to collaborate with or consider another subsystem for a particular action (e.g. Stingers shall lower at the same rate as MonkeyBars to horizontally lift the robot to the third stage).<br>

| Requirement Type | Subsystem | Component | Interfaces with | Description |
|------------------|-----------|-----------|-----------------|-------------|
| Interlock | Drivetrain (D) |  | Lift (L) | the max drivetrain speed must be reduced when the lift is above a certain height |
| Command | Drivetrain:DriveWithJoysticks | _leftMotor1, _leftMotor2, _leftMotor3, _rightMotor1, _rightMotor2, _rightMotor3, _gyroSPI, _imu, _drivetrainPosController, _rotateDriveStraightController,  | Drivetrain (D) | Gives control of the drivetrain with a joystick using varying control styles |
| External | Drivetrain (D) | _drivetrainShifter | Stingers, DrivetrainStingerShifter | during endgame the motor power is transferred from the drivetrain to the stingers and then back again using the DrivetrainStingerShifter  |

<img src="http://yuml.me/diagram/scruffy/class/[Drivetrain |-SpeedController _leftMotor1; -SpeedController _leftMotor2;-SpeedController _leftMotor3;-SpeedController _rightMotor1; -SpeedController _rightMotor2;-SpeedController _rightMotor3; -ADXRS453Gyro _gyroSPI;
    -AverageEncoder _drivetrainLeftEncoder; -AverageEncoder _drivetrainRightEncoder; +double _leftMotor1Voltage; +double _leftMotor2Voltage; +double _leftMotor3Voltage; +double _rightMotor1Voltage; +double _rightMotor2Voltage; +double _rightMotor3Voltage; +IMU _imu; +PIDPosition _drivetrainPosController; +PIDPosition _rotateDriveStraightController; -Drivetrain _instance |-Drivetrain(); +getInstance(); +driveLeftMotor1(double speed); +driveLeftMotor2(double speed); +driveLeftMotor3(double speed); +driveRightMotor1(double speed); +driveRightMotor2(double speed); +driveRightMotor3(double speed); +driveLeftMotors(double speed); +driveRightMotors(double speed); +dangerousTankDrive(double leftSpeed  double rightSpeed); +tankDrive(double leftSpeed double rightSpeed); +getHeading(); +resetGyro(); +calibrateGyro(); +startGyroCalibrating(); +isGyroCalibrated; +isGyroCalibrating; +stopGyroCalibrating(); +getRightPosition(); +getLeftPosition(); +getAverageDistance();+resetRightPosition(); +resetLeftPosition(); +resetPosition(); +getLeftMotor1Voltage(); +getLeftMotor2Voltage(); +getLeftMotor3Voltage(); +getRightMotor1Voltage(); +getRightMotor2Voltage(); +getRightMotor3Voltage(); +getRightEncoderRate(); +getLeftEncoderRate(); +getAverageEncoderRate(); +PIDVoltageFeedLeftMotor1(); +PIDVoltageFeedLeftMotor2(); +PIDVoltageFeedLeftMotor3(); +PIDVoltageFeedRightMotor1(); +PIDVoltageFeedRightMotor2(); +PIDVoltageFeedRightMotor3(); +initDefaultCommand()]" >
 
### DrivetrainStingerShifter (DSS)
Serves dual function of climbing to 3rd level on endgame and intaking cargo during match. This system will need to work in tandem with the stingers in order to lift the robothorizontally onto the 3rd level.

| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| DSS01 | private | DoubleSolenoid | _drivetrainShifter | Changes gear engaging or disengaging the drivetrain and the stingers |
| DSS02 | private | DrivetrainStingerShifter | _instance | object designed to create a singleton object of the DrivetrainStingerShifter |


#### Requirements: Interlocks, Functionality, or External
**Interlocks** represent scenarios where the robot may be capable of harming itself by its own subsystem action or by the action of another subsystem (e.g. Intake may not lift higher than the crossbar of the lift when it is rotated towards the back of the robot).<br>
**Functionality** is a capability the subsystem needs to have (e.g. The intake shall hold the cargo in place until it is commanded to shoot or drop the cargo).<br>
**External** represent scenarios when a subsystem will need to collaborate with or consider another subsystem for a particular action (e.g. Stingers shall lower at the same rate as MonkeyBars to horizontally lift the robot to the third stage).<br>

| Requirement Type | Subsystem | Component | Interfaces with | Description |
|------------------|-----------|-----------|-----------------|-------------|
| Command | DrivetrainStingerShifter(DSS): EngageDrivetrain | _drivetrainShifter | Drivetrain (D), Stingers (S) | engages the drivetrain and places the stingers in neutral |
| Command | DrivetrainStingerShifter(DSS): EngageStingers | _drivetrainShifter | Drivetrain (D), Stingers (S) | engages the stingers and places the drivetrain in neutral |
| Warning | DrivetrainShifterStinger | _drivetrainShifter | Drivetrain (D) | Make sure drivetrain is engaged at the beginning of the match |
| Functionality + External | DrivetrainStingerShifter (DSS) | _drivetrainShifter  | Drivetrain (D), Stingers (S) | During end game, the shifter will have to engage the stingers to raise the robot, and then re-engage the drivetrain to pull the chassis onto the 3rd stage of the HAB  |

<img src="http://yuml.me/diagram/scruffy/class/[DrivetrainStingerShifter|-DoubleSolenoid _drivetrainShifter;-DrivetrainStingerShifter _instance |-DrivetrainStingerShifter(); +getInstance(); +engageDrivetrain(); +engageStingers(); +isDrivetrainEngaged(); +isStingerEngaged(); +initDefaultCommand()]" >

### PlungerArmPivot (PAP)
Rotates the hatch plunger arm from the front of the robot to the back and vice versa. This allows the hatch panels to be grabbed on one side from the human player station and scored on the other. It has to be controlled so the pivot will only rotate when it will not hit the crossbar of the lift. 

| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| PAP01 | private | VictorSP | _plungerArmPivotMotor | powers the arm to rotate 180 degrees from front to back and vice versa |
| PAP02 | private | AveragePotetiometer | _pivotPot | gives the rotary position of the plunger arm |
| PAP03 | private | PlungerArmPivot | _instance | object designed to create a singleton object of the PlungerArmPivot |


#### Requirements: Interlocks, Functionality, or External
**Interlocks** represent scenarios where the robot may be capable of harming itself by its own subsystem action or by the action of another subsystem (e.g. Intake may not lift higher than the crossbar of the lift when it is rotated towards the back of the robot).<br>
**Functionality** is a capability the subsystem needs to have (e.g. The intake shall hold the cargo in place until it is commanded to shoot or drop the cargo).<br>
**External** represent scenarios when a subsystem will need to collaborate with or consider another subsystem for a particular action (e.g. Stingers shall lower at the same rate as MonkeyBars to horizontally lift the robot to the third stage).<br>

| Requirement Type | Subsystem | Component | Interfaces with | Description |
|------------------|-----------|-----------|-----------------|-------------|
| Command | PlungerArmPivot:DrivePlungerArmPivotWithConstant | _plungerArmPivotMotor | Hatch Plunger (P) | sends a constant speed to the pivot (based into the command during instantiation in OI) |
| Command | PlungerArmPivot:DrivePlungerArmPivotWithJoysticks | _plungerArmPivotMotor | Hatch Plunger (P) | sends a speed from an axis of the operator joystick to the pivot (determined in OI method getDrivePlungerArmPivotJoystickValue()) |
| Interlock | PlungerArmPivot | _plungerArmPivotMotor, _pivotPot, _plungerArmBrake | Lift (L), PlungerArmHardStop (PAHS) | If Pivot turns when the lift is at one of several wrong heights (still TBD), the pivot will hit the crossbar of the lift |
| Functionality + External | PlungerArmPivot | _plungerArmPivotMotor | HatchPlunger (P) | The pivot controls the rotational position of the hatch plunger, which has to deliver hatch panels to the rocket and cargo ship, as well as receive panels from the human player station  |
| Functionality + External | PlungerArmPivot | _plungerArmPivotMotor | HatchPlunger (P), FloorIntake (FI) | The pivot controls the rotational position of the hatch plunger, which has receive hatch panels from the floor intake |

<img src="http://yuml.me/diagram/scruffy/class/[PlungerArmPivot |-VictorSP _plungerArmPivotMotor; -AveragePotentiometer _pivotPot; -PlungerArmPivot _instance |-PlungerArmPivot(); +getInstance(); +drivePlungerArmPivotMotor(double speed); +getRawPot(); +getPotPos(); +initDefaultCommand()]" >

### PlungerArmHardStop (PAHS)
Hard stop for the Plunger Arm Pivot. Probably involved in cut-offs for interlock preventing the plunger arm from turning into the lift (see above).

| ID | Scope | Type | Name | Description |
|----|-------|------|------|-------------|
| PAHS01 | private | DoubleSolenoid | _plungerArmBrake | hard stop for the plunger arm pivot |
| PAHS02 | private | PlungerArmHardStop | _instance | object designed to create a singleton object of the PlungerArmHardStop |


#### Requirements: Interlocks, Functionality, or External
**Interlocks** represent scenarios where the robot may be capable of harming itself by its own subsystem action or by the action of another subsystem (e.g. Intake may not lift higher than the crossbar of the lift when it is rotated towards the back of the robot).<br>
**Functionality** is a capability the subsystem needs to have (e.g. The intake shall hold the cargo in place until it is commanded to shoot or drop the cargo).<br>
**External** represent scenarios when a subsystem will need to collaborate with or consider another subsystem for a particular action (e.g. Stingers shall lower at the same rate as MonkeyBars to horizontally lift the robot to the third stage).<br>

| Requirement Type | Subsystem | Component | Interfaces with | Description |
|------------------|-----------|-----------|-----------------|-------------|
| Command | PlungerArmHardStop:EnablePlungerArmBrake | _plungerArmBrake | PlungerArmPivot (PAP) | engages the brake to stop the plunger arm pivot |
| Command | PlungerArmHardStop:DisablePlungerArmBrake | _plungerArmBrake | Hatch Plunger (P) | disengages brake to re-allow movement to the plunger arm pivot |
| Functionality + External | PlungerArmHardStop | _plungerArmBrake | PlungerArmPivot (PAP) | The brake stops the movement of the PlungerArmPivot  |

<img src="http://yuml.me/diagram/scruffy/class/[PlungerArmHardStop |-DoubleSolnoid _plungerArmBrake; -PlungerArmHardStop _instance |-PlungerArmHardStop(); +getInstance(); +enablePlungerArmBrake; +disablePlungerArmBrake; +isEnabledPlungerArmBrake; +isDisabledPlungerArmBrake; +initDefaultCommand()]" >

