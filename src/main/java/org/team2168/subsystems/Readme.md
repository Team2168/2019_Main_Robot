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
| MB01 | private | VictorSP | _rightRaiseMotor | Range of -25 deg to 90 deg from vertical |
| MB02 | private | VictorSP | _leftRaiseMotor | Range of -25 deg to 90 deg from vertical |
| MB03 | private | VictorSP | _leftWheels | Active intake on wheels and drive robot onto third level |
| MB04 | private | VictorSP | _rightWheels | Active intake on wheels and drive robot onto third level |
| MB05 | private | DoubleSolenoid | _brake | Break to hold Monkey Bar arms in a fixed position |
| MB06 | private | DigitalInput | _isFullyRaised | Boolean sensor to indicate if MB is raised fully or not |
| MB07 | private | AverageEncoder | _currPosition | Rotary position sensor (Potentiometer) to indicate the current position of the MB |

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