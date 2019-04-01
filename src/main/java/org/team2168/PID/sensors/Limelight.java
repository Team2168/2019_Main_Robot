package org.team2168.PID.sensors;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight implements PIDSensorInterface
{

    // Target position and camera settings
    private NetworkTable networkTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry camtran;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry camMode;
    private NetworkTableEntry pipeline;

    private double currentPosition;
    private double previousPosition;

    private boolean variablesInstantiated;

    private double runTime;
    private int averagorSize;
    private double[] averagorArray;
    private int arrayPos = 0;

    /**
     * Default constructor
     */
    public Limelight()
    {
        currentPosition = 0.0;
        previousPosition = 0.0;
        /**
         * Check networkTable to verify network connectivity of Limelight
         */
        
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        variablesInstantiated = false;

        runTime = Timer.getFPGATimestamp();
        averagorSize = RobotMap.LIMELIGHT_AVG_ENCODER_VAL;
        averagorArray = new double[averagorSize];

        this.instantiateLocalVariables();

        // Testing only
         ConsolePrinter.putNumber("Vision Target Bearing", () -> {return Robot.drivetrain.limelight.getPos();}, true, false);
        // ConsolePrinter.putNumber("Vision Target Area", () -> {return Robot.drivetrain.limelight.getTargetArea();}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 1", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[0];}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 2", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[1];}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 3", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[2];}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 4", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[3];}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 5", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[4];}, true, false);
        // ConsolePrinter.putNumber("Vision Target Position 6", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[5];}, true, false);
        // ConsolePrinter.putNumber("Limelight Controller Output", () -> {return Robot.drivetrain.limelightPosController.getControlOutput();}, true, false);
        // ConsolePrinter.putNumber("Limelight P", () -> {return RobotMap.LIMELIGHT_POSITION_P;}, true, true);
        // ConsolePrinter.putNumber("Limelight I", () -> {return RobotMap.LIMELIGHT_POSITION_I;}, true, true);
        // ConsolePrinter.putNumber("Limelight D", () -> {return RobotMap.LIMELIGHT_POSITION_D;}, true, true);
    
        if(this.connectionEstablished())
        {
            this.instantiateLocalVariables();
        }
        else
        {
            SmartDashboard.putBoolean("IsLimeLightPresent", false);
        }
    }

     /**
     * Returns the rate at which the target bearing changes
     */
    @Override
    public double getRate()
    {
        double executionTime = Timer.getFPGATimestamp() - runTime;
        double pos = getPos();
        if(executionTime > 0) {
            putData((pos - previousPosition) / executionTime);
        }

        runTime = Timer.getFPGATimestamp();
        previousPosition = pos;
        return getAverage();
    }

    @Override
    public void reset()
    {
        // Does not apply
    }

    /**
     * Returns the target bearing
     * @return is a double from -27.0 to 27.0
     */
    @Override
    public double getPos()
    {
        previousPosition = currentPosition;
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            currentPosition = tx.getDouble(0.0);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            this.currentPosition = tx.getDouble(0.0);
        }
        else
        {
            this.currentPosition = 0.0;
        }
        
        return this.currentPosition;
    }

    /**
     * Returns the target's area, measured by the percentage of the frame it takes up
     * @return is a double from 0.0 to 100.0
     */
    public double getTargetArea()
    {
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return ta.getDouble(0.0);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return ta.getDouble(0.0);
        }
        else
        {
            return 0.0;
        }
    }

    /**
     * Returns the translation of the camera from the target
     * @return is an array of 6 doubles; gives the translation (x,y,z) and rotation (pitch,yaw,roll)
     */
    public double[] getCameraTranslation()
    {
        double[] defaultValues = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return camtran.getDoubleArray(defaultValues);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return camtran.getDoubleArray(defaultValues);
        }
        else
        {
            return defaultValues;
        }
        
        
    }

    public void setCamMode(int camModeNumber)
    {
        if(camModeNumber >= 0 && camModeNumber <= 2)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                camMode.setNumber(camModeNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                camMode.setNumber(camModeNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }

    public int getCamMode()
    {
        if(this.connectionEstablished() && this.variablesInstantiated)
        {
            return camMode.getNumber(1).intValue();
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return camMode.getNumber(1).intValue();
        }
        else
        {
            //System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            return -1; 
        }
    }

    /**
     * Sets the pipeline being used
     * @param pipelineNumber is an int from 0 to 9
     */
    public void setPipeline(int pipelineNumber)
    {
        if(pipelineNumber >= 0 && pipelineNumber <= 9)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                pipeline.setNumber(pipelineNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                pipeline.setNumber(pipelineNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }
    
    /**
     * Returns the current pipeline number
     * @return is an int from 0 to 9
     */
    public int getPipeline()
    {
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return pipeline.getNumber(0).intValue();
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return pipeline.getNumber(0).intValue();
        }
        else
        {
            //System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            return -1;
        }
        
    }

    /**
     * Sets the LED mode
     * @param ledNumber is an int from 0 to 3
     *                  0 - use the LED Mode set in the current pipeline
     *                  1 - force off
     *                  2 - force blink
     *                  3 - force on
     */
    public void setLedMode(int ledNumber)
    {
        if(ledNumber >= 0 && ledNumber <= 3)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                ledMode.setNumber(ledNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                ledMode.setNumber(ledNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }

    private boolean connectionEstablished()
    {
        //return this.networkTable.containsKey("tx");
        return !(this.networkTable.getEntry("tx") == null);
    }

    private void instantiateLocalVariables()
    {
        SmartDashboard.putBoolean("IsLimeLightPresent", true);
            
        // Variables to get data from Limelight
        tx = networkTable.getEntry("tx");
        ta = networkTable.getEntry("ta");
        camtran = networkTable.getEntry("camtran");

        // Variables to set data on Limelight
        ledMode = networkTable.getEntry("ledMode");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        // Sets the camera controls
        ledMode.setNumber(0);
        camMode.setNumber(1);
        pipeline.setNumber(0);

        this.variablesInstantiated = true;
    }

      /**
	 * Puts data in to array to be averaged, hence the class name and method name.
	 * Its like magic but cooler.
	 *
	 * @param value the value being inserted into the array to be averaged.
	 */

    public synchronized void putData(double value) {
        averagorArray[arrayPos] = value;
        arrayPos++;

        if(arrayPos >= averagorSize) {
            // Is equal or greater to averagorSize because array is zero indexed. Rolls over index position
            arrayPos = 0;
        }
    }

    /**
	 * Returns average of last n values sent, as name says.
	 *
	 * @return the average
	 */
    public synchronized double getAverage() {
        double sum = 0;
        for(int i = 0; i < averagorSize; i++) {
            sum += averagorArray[i];
        }
        return sum / averagorSize;
    }


}
