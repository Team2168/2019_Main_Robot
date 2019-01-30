# Getting Started with VS Code for 2019 Season

Directions can be found here: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/1027503-installing-c-and-java-development-tools-for-frc

Brief Synopsis for windows:
1. Uninstall any VS code installattion from your computer (This just makes things easier)
2. download and install .net: https://support.microsoft.com/en-us/help/3151800/the-net-framework-4-6-2-offline-installer-for-windows
3. Download the FRC VS Code installer for your operating system from here: https://github.com/wpilibsuite/allwpilib/releases
4. Extract the above installer and run it for all users
5. Use the installer to download and install VS code (top right button)
6. Make sure all checkboxes are checked then run the execute install (bottom center button)
7. install git command line on your computer https://git-scm.com/downloads
8. Launch git bash application
9. In git bash type git
9. Open VS Code using the FRC shortcut on your desktop, underextensions, install 'git lens' extension in VS Code


## Clone this Repo

1. In VS Code press `ctrl+shift+p`
2. In command dropdown type `git clone' and hit enter
3. Enter repo URL `https://github.com/Team2168/2019_Main_Robot.git` and hit enter
4. Select a Folder to store the files in (Make a new folder) for all robotics like: `C:\Users\2168\Documents\RoboticsDevelopment`
5. A window will pop up to login, enter your github login credentials
6. The code will download into the folder above
7. A pop up will ask if you would like to open the project, select yes
8. If any pop-ups show up to update WPI Lib, select yes

# Common tasks in VS Code
## Videos
Videos for the above can be found at the following playlist
https://www.youtube.com/playlist?list=PLUTJdMwEWueIyWRVWkQE8N3XxPGucEx0Q

## Setup git credentials
1. Open Git bash
2. Set global name to your name type `git config --global user.name "Kevin Harrilal"
3. Set global email to your email type `git config --global user.name "Kevin@team2168.org"

## To pull the latest code
1. In VS Code press `ctrl+shift+p`
2. Type `git Fetch`

## To Commit
1. Use the source control pane
2. Add files to the commit using the +
3. Add a commit message and hit the check symbol
4. Push using the menu

## To build the code
1. In VS Code press `ctrl+shift+p`
2. Type `Wpilib build`

## To deploy the code
1. In VS code press `ctrl+shift+p`
2. Type `WPILib deploy`



# 2019_Main_Robot
Code for the [FIRST DeepSpace](https://www.youtube.com/watch?v=Mew6G_og-PI) game. This readme provide all of the information required to get started and programming for the 2019 season. 

## Requirements for Robot
1. RoboRio must be flashed to latest 2018 image using USB (This only needs to be done once for the season): (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/144984-imaging-your-roborio)
2. Radio must be programed (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/144986-programming-your-radio)
3. Roborio IP on ethernet must be set to static using web dashboard to : 10.21.68.2
4. Any IP Camera must be set to 10.21.68.90
5. Vision Processor: If Tegra must be set to 10.21.68.71 (for "TK1"), if Beablebone must be set to 10.21.68.33 (for "BB"), if android must be set to 10.21.68.46 (for "AD")

## Requirements for Students

Screen Steps Live is the official documentation from FRC on the control system: Please give it a read: https://wpilib.screenstepslive.com/s/4485

1. In order to program you need to set up your Java development environment using VS Code instructions above. If you would like to use eclipse, KH hasen't yet ran through that setup so proceed at your own risk. (I will update once I run through eclipse, but done hold your breath)
2. If you would like to have the driverstation on your computer as well then install NI Update suite, but this is not a requirement to develop or deploy programs, only to flash robot images (https://wpilib.screenstepslive.com/s/currentCS/m/java/l/1027504-installing-the-frc-update-suite-all-languages)
4. For returning students, and new students interested: understand what has changed in WPI Librarary since 2018 season (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/1028812-new-for-2019)
5. Understand how the robot is wired as it affects your code. (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/599673-wiring-the-frc-control-system)
## Cool things to know

## Everything you need to know about the control system is here:
1. More information on the control system can be found at our controls website at http://controls.team2168.org

### Radio
1. You can access radio web page by logging into http://10.21.68.1 root/admin
2. Roborio should always be plugged into the port on the radio labeled "18-24 vPOE" only! Or connect to this port via a switch

### Roborio
1. You can access roborio diagnostics webpage by http://roboRIO-2168-FRC.local (using IE web browser) or http://10.21.68.2
2. You can program roborio over ethernet, usb, or wifi (if USB, NI Update suite needs to be installed to get usb drivers)
3. More information on the control system can be found at our controls website at http://controls.team2168.org
4. Files will be logged to /home/lvuser/Logs
5. You can ftp files to/from the roborio using filezilla, winscp, web browser, or your local file explorer at ftp://10.21.68.2:21
6. You can ssh into roborio using putty or console application at ssh 10.21.68.2:22 username:lvuser password: blank

### Dashboard (on driver station)
1. Java dashboard will open if Java is selected from the driverstation menu
2. Python dash (if it installed) will open if "default" dashboard is selected from drivestation menu
3. If smartdashboard doesn't update, but you have robot comms, in smart dash preferences toggle "use mDNS" until it does. 

#Repository Guidelines
##Branches
Our repository and workflow loosely follows the gitflow workflow. This workflow is simple and is one of the most popular workflows when using git with a large number of developers. More info: https://www.atlassian.com/git/tutorials/comparing-workflows#gitflow-workflow
- The master branch contains code that is known-working, has been tested, and can be deployed to a competition ready robot.
- The develop branch is our sandbox for integrating and testing new features and fixing problems. This isnt the latests and greatest code, but it may have problems and needs to be checked out on the robot before being pushed into master. 
- Everything else is lumpped under feature/bugfix branches. When we need to add new capabilities, start by branching the latest code  in the develop branch.  

## Checklist for committing/pushing code
- Commit often and create detailed log messages that describe what and why you're making a change. Be specific.
- Review the changes you make before pushsing them. You should look through all the files being added/modified/removed before you commit.
- Always verify your code compiles before pushing to the repo. There shouldn't be any red-underlined text in your commits. Use the build button (Green triangle) at the top of eclipse to verify a build completes without error.
- Push your changes into a branch with a name that identifies what feature it is adding or problem it is addressing in the code.
- Never push to the master branch 
- After pushing your changes to the repo, verify you can see your changes in GitHub from a web browser.

#Robot Design (To be continued)
- All students are assigned one or many subsystems on the robot. Your task is the following:
1. Pull master and create a new branch (naming it after the subsystem your working on, followed by your initials is a good idea)
2. Write the subsystem with all the hardware called out below.
3. RobotMap ports may be pre-determined and assigned to your subsystem, but you need to add the code to RobotMap java
4. Write all the commands for your subsystem, and place them in the approporate command subfolders/packages for your subsystem
5. Add the subsystem to robot.java
6. Push your code in a new banch for review by Kevin or James (we will write issues for you to fix)
7. Once tested on a robot, it will be merged into Master. 

## Subsystems
### Drivetrain(Alyssa)
- 6x VictorSP Motor controllers (use speed controller class so we can switch them on the fly)
..- Positive values move upwards / Negative values move down
- 2x Greyhill Encoders (1x left, 1x right)
- 1x Gyro (ADXRS453) (may change to Nav X)
- use a digital input to control practice bot
- use a digital input to control if CAN or PWM 

### Plunger Arm Pivot *Unassigned*
- 1x Victor SP motor controller
- 1x Double Solenoid for brake
- 1x AveragePot for position feedback

### Cargo Intake/Claw (Jean-Carlos)
- 2x VictorSP motor controllers
..- Positive values move cube inwards / Negative values move cube outwards
- 1x Double Solenoid valve to punch ball
..- Koff is intake open / Kon is intake closed
- 1x SHARP IR sensor to detect the presence of a cargo

### Hatch Plunger (Young Aiden)
- 1x Double Solenoid to extend/retract the plunger
..- Kforward is extended, Kreverse is retract
- 1x Double Solenoid to extend/retract the plunger
..- Kforward is hatch engaged, Kreverse is hatch disengaged
- 1 AveragePot for position feedback
- 1x SHARP IR sensor to detect the presence of a hatch

### Lift(Nathan)
- 2x VictorSP motor controllers to drive lift up/down
..- Positive values move upwards / Negative values move down
..- On 30A fuses on the PDP
- 2x hall effect sensors (discrete inputs) for fully raised & fully lowered position indications 
- 1x pneumatic Double Solenoid for brake
..- KForward is high speed / KReverse is low speed
- 1x Encoder/10 turn potentiometer for lift position

### Floor hatch mechanism (Conor)
- 1x Double Solenoid to rotate up and down the mechanism
- 1x Victor SP for the wheels
- 2x hall effect digital IO for Up/Down position

### Moneky Bar (Liam)
- 2x Victor SP for Motor wheels
- 2x Victor SP for Rotation (in opposite directions maybe)
- 2x AveragePot for position

### Stinger (Kaleb)
- 2x Victor SP for winch
- 2x averagePot for positon
- 2x hall effects for retracted detection


## Vision (Cierra)
- Track target
- Stitch camera feeds to create panoramic view

## Controls (Old Aiden)
- TBD





