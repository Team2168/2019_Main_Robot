package org.team2168.PID.sensors;

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    /**
     * Default constructor
     */
    public Limelight()
    {
        // Variables to get data from Limelight
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = networkTable.getEntry("tx");
        ta = networkTable.getEntry("ta");
        camtran = networkTable.getEntry("camtran");

        currentPosition = 0.0;
        previousPosition = 0.0;

        // Variables to set data on Limelight
        ledMode = networkTable.getEntry("ledMode");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        // Sets the camera controls
        ledMode.setNumber(0);
        camMode.setNumber(0);
        pipeline.setNumber(0);

        // Testing only
        ConsolePrinter.putNumber("Vision Target Bearing", () -> {return Robot.drivetrain.limelight.getPos();}, true, false);
        ConsolePrinter.putNumber("Vision Target Area", () -> {return Robot.drivetrain.limelight.getTargetArea();}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 1", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[0];}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 2", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[1];}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 3", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[2];}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 4", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[3];}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 5", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[4];}, true, false);
        ConsolePrinter.putNumber("Vision Target Position 6", () -> {return Robot.drivetrain.limelight.getCameraTranslation()[5];}, true, false);
        ConsolePrinter.putNumber("Limelight Controller Output", () -> {return Robot.drivetrain.limelightPosController.getControlOutput();}, true, false);
        ConsolePrinter.putNumber("Limelight P", () -> {return RobotMap.LIMELIGHT_POSITION_P;}, true, true);
        ConsolePrinter.putNumber("Limelight I", () -> {return RobotMap.LIMELIGHT_POSITION_I;}, true, true);
        ConsolePrinter.putNumber("Limelight D", () -> {return RobotMap.LIMELIGHT_POSITION_D;}, true, true);
    }

    /**
     * Returns the rate at which the target bearing changes
     */
    @Override
    public double getRate()
    {
        return Math.abs((previousPosition - currentPosition) / previousPosition);
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
        currentPosition = tx.getDouble(0.0);
        return currentPosition;
    }

    /**
     * Returns the target's area, measured by the percentage of the frame it takes up
     * @return is a double from 0.0 to 100.0
     */
    public double getTargetArea()
    {
        return ta.getDouble(0.0);
    }

    /**
     * Returns the translation of the camera from the target
     * @return is an array of 6 doubles; gives the translation (x,y,z) and rotation (pitch,yaw,roll)
     */
    public double[] getCameraTranslation()
    {
        double[] defaultValues = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return camtran.getDoubleArray(defaultValues);
    }

    /**
     * Sets the pipeline being used
     * @param pipelineNumber is an int from 0 to 9
     */
    public void setPipeline(int pipelineNumber)
    {
        if(pipelineNumber >= 0 && pipelineNumber <= 9)
        {
            pipeline.setNumber(pipelineNumber);
        }
    }
    
    /**
     * Returns the current pipeline number
     * @return is an int from 0 to 9
     */
    public int getPipeline()
    {
        return pipeline.getNumber(0).intValue();
    }

}