package org.team2168.PID.controllers;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimerTask;

import org.team2168.PID.sensors.PIDSensorInterface;
import org.team2168.utils.TCPMessageInterface;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Kevin Harrilal, First Robotics Team 2168
 * 
 *         The PID Speed class implements a PID controller used to perform speed
 *         control on a DC motor. The purpose of this class is to keep a DC
 *         motor rotating at a constant speed when the correct P, I, and D gains
 *         have been chosen. <br>
 *         <br>
 *         The controller implements the parallel form the PID controller. <br>
 *         <br>
 *         In addition to a parallel form PID Controller. This class also
 *         implements the following features: Derivative Filtering, Integral
 *         Windup Prevention, Gain Scheduling, Dead Band Removal, Error
 *         Tolerance, Output Coercion, and many other features to allow for a
 *         stable controller. <br>
 *         <br>
 *         This class was intended to work with the 2012 FRC Java Library
 *         developed by WPI. This class will not run if the WPI libraries are
 *         not installed on the local machine. <br>
 *         <br>
 *         <u>Precondition:</u> An Encoder of the WPI class is instantiated.
 *         Proportial, Derivative, and Integral gains are set, and a period in
 *         miliseconds in known. <br>
 *         <br>
 *         <u>PostCondition:</u> Once Constructed, a new thread will be spawned
 *         which will have a periodic execution rate as specified by the period
 *         value. The new thread will be paused and a call to the Objects
 *         start() method will allow the thread to start executing the PID loop.
 *         <br>
 *         <br>
 *         To use this class is simple, an example is below <br>
 *         <br>
 * 
 *         Encoder leftEncoder= new Encoder(1,2) //Encoder on DIO ports 1 and 2
 *         <br>
 *         double P = 1; //P gain <br>
 *         double I = 2; //I gain <br>
 *         double D = 3; //D gain <br>
 *         long period = 40; // 40 millisecond period (note: Type is Long) <br>
 *         <br>
 *         leftEncoder.start(); //start the encoder<br>
 *         leftEncoder.reset(); //reset the encoder (not needed but useful) <br>
 *         <br>
 *         PIDSpeed speedController = new PIDSpeed("DriveTrain Speed
 *         Controller", P, I, D, leftEncoder, period); //launch the PID thread
 *         <br>
 *         speedController.Enable(); //start the PID thread<br>
 *         <br>
 *         Multiple instances of the PIDSpeed Object can be created for multiple
 *         PID loops to run. Each loop will run in its own thread at the period
 *         specified in its constructor. <br>
 */
public class PIDSpeed implements TCPMessageInterface {
	// gains for gain schedule
	private volatile double pGain;
	private volatile double iGain;
	private volatile double dGain;

	private volatile double pGain2;
	private volatile double iGain2;
	private volatile double dGain2;

	// internal PID variables
	private volatile double p;
	private volatile double i;
	private volatile double d;

	// enable
	private volatile boolean enable;

	// isFinished
	private volatile boolean isFinished;

	// create local PID portions
	double prop;
	double integ;
	double deriv;

	// Other internal variable
	private volatile boolean enGainSched;
	private volatile boolean enDerivFilter;

	// internal calcs
	private volatile double err; // error
	private volatile double olderr; // oldError
	private volatile double sp; // setpoint
	private volatile double cp; // current position
	private volatile double co; // control output
	private volatile double coNotSaturated; // control output unmodified for
											// graphing
	private volatile double coOld; // Control Output of last iteration
	private volatile double errsum; // total of all errors this loop iteration
	private volatile double olderrsum; // total of all errors last loop
										// iteration

	// timers
	private volatile double clock;
	private volatile double executionTime;
	private volatile double runTime;

	// deriv filters
	private volatile double filterDerivOld;
	private volatile double r; // between 0 and 1

	// max and min limit variables
	private volatile double maxPosOutput; // max positive output (+1)
	private volatile double maxNegOutput; // max negative output (-1)
	private volatile double minPosOutput; // min positive output, use to get rid
											// of deadband
	private volatile double minNegOutput; // min negative output, use to get rid
											// of deadband

	// acceptable steadyState error
	private volatile double acceptErrorDiff; // allowable error (in units of
												// setpoint)

	private double diff;

	// tread executor
	java.util.Timer executor;
	long period;

	// encoder
	PIDSensorInterface encoder = null;

	// Name of Thread
	private volatile String name;

	// Variables to Determine ifFinished
	private volatile int SIZE;
	private volatile double[] atSpeed;
	private volatile int count;

	// variables to command setpoints with array
	boolean setPointByArray;
	int setPointArrayCounter;
	double[] setPointArray;

	private double int_d_term;
	private double lastDeriv;
	private volatile double n;

	PrintWriter log;

	/**
	 * This is the default constructor for the {@link PIDSpeed} class. All other
	 * constructors within this class make a call to this constructor first.
	 * 
	 * 
	 * 
	 * @param name
	 *            - type String used to identify this PID Instance and thread i.e
	 *            "LeftDrivePID"
	 * @param P
	 *            - type double which represents Proportional Gain for the Speed
	 *            Controller
	 * @param I
	 *            - type double which represents Integral Gain for the Speed
	 *            Controller
	 * @param D
	 *            - type double which represents Derivative Gain for the Speed
	 *            Controller
	 * @param N
	 *            - type double which represents Derivative Gain Filter between 0 and 1
	 * @param currentPos
	 *            - type SpeedSensorInterface Object which is used to reference the
	 *            encoder object this PID loop will use as feedback
	 * @param period
	 *            - type long which represents the time the thread will execute at
	 *            in milliseconds. i.e 40 means the loop will execute every 40ms.
	 * 
	 * @throws NullPointerException
	 *             if the Speed Sensor object passed is null;
	 */
	public PIDSpeed(String name, double P, double I, double D, double N, PIDSensorInterface currentPos, long period) {

		if (currentPos == null)
			throw new NullPointerException("Speed Sensor Object of " + name + " is null");

		// copy values
		this.name = name;
		this.pGain = P;
		this.iGain = I;
		this.dGain = D;
		this.encoder = currentPos; // point to reference of encoder instead of
									// creating new
		this.period = period;
		this.pGain2 = P;
		this.iGain2 = I;
		this.dGain2 = D;

		// disable PID loop
		this.enable = false;

		// zero all other parameters
		this.acceptErrorDiff = 0;
		this.clock = 0;
		this.runTime = 0;
		this.co = 0;
		this.coNotSaturated = 0;
		this.coOld = 0;
		this.cp = 0;
		this.p = 0;
		this.i = 0;
		this.d = 0;
		this.prop = 0;
		this.deriv = 0;
		this.integ = 0;
		this.err = 0;
		this.errsum = 0;
		this.filterDerivOld = 0;
		this.olderr = 0;
		this.olderrsum = 0;
		this.n = N;
		this.sp = 0;

		// set Output Limits
		this.maxNegOutput = -1;
		this.maxPosOutput = 1;
		this.minNegOutput = 0;
		this.minPosOutput = 0;

		// set all booleans to false

		this.enDerivFilter = false;
		this.enGainSched = false;
		this.isFinished = false;

		// at speed size
		this.SIZE = 1;
		this.atSpeed = new double[SIZE];
		this.count = 0;

		// reset encoder
		this.encoder.reset();

		// setpoint array
		setPointByArray = false;
		setPointArrayCounter = 0;
		setPointArray = null;

		this.diff = 0;

		try {

			Date date = new Date();
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss");

			File file = new File("/home/lvuser/Logs/PID/Speed");
			if (!file.exists()) {
				if (file.mkdirs()) {
					System.out.println("Log Directory is created!");
				} else {
					System.out.println("Failed to create Log directory!");
				}
			}

			this.log = new PrintWriter(
					"/home/lvuser/Logs/PID/Speed/" + dateFormat.format(date) + "-" + this.name + "-Log.txt", "UTF-8");
			this.log.println(
					"time: \tTimeOfDay: \tcp: \tsp: \terr: \tpterm: \twindup: \terrsum: \titerm: \tdterm: \toutput: \tcounstat: \texctime:");
			this.log.flush();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * This constructor for the {@link PIDSpeed} class allows the user to set PID
	 * gains for gainScheduling.<br>
	 * <br>
	 * This is handy for when one would like to have separate gains when the Error
	 * between the setpoint and the CurrentValue is Positive verse Negative. For
	 * example this would be useful if one wished to have separate gains to go
	 * forward and reverse on a drivetrain.<br>
	 * <br>
	 * This constructor also instantiates the new thread for the PID loop will run
	 * in. Although the PID loop thread has been created, the PID loop will not
	 * start running until a call to enable() has been made.
	 * 
	 * 
	 * @param name
	 *            - type String used to identify this PID Instance and thread i.e
	 *            "LeftDrivePID"
	 * @param pUp
	 *            - type double which represents Proportional Gain for the Speed
	 *            Controller to use when the Error is greater than zero.
	 * @param iUp
	 *            - type double which represents Integral Gain for the Speed
	 *            Controller to use when the Error is greater than zero.
	 * @param dUp
	 *            - type double which represents Derivative Gain for the Speed
	 *            Controller to use when the Error is greater than zero.
	 * @param pDown
	 *            - type double which represents Proportional Gain for the Speed
	 *            Controller to use when the Error is less than zero.
	 * @param iDown
	 *            - type double which represents Integral Gain for the Speed
	 *            Controller to use when the Error is less than zero.
	 * @param dDown
	 *            - type double which represents Derivative Gain for the Speed
	 *            Controller to use when the Error is less than zero.
	 * 
	 * @param currentPos
	 *            - type Encoder Object which is used to reference the encoder
	 *            object this PID loop will use as feedback
	 * @param period
	 *            - type long which represents the time the thread will execute at
	 *            in milliseconds. i.e 40 means the loop will execute every 40ms.
	 * 
	 * @throws NullPointerException
	 *             if the Speed Sensor object passed is null;
	 */

	public PIDSpeed(String name, double pUp, double iUp, double dUp, double nUp, double pDown, double iDown,
			double dDown, PIDSensorInterface currentPos, long period) {
		this(name, pUp, iUp, dUp, nUp, currentPos, period);
		this.pGain2 = pDown;
		this.iGain2 = iDown;
		this.dGain2 = dDown;
		this.enGainSched = true;

	}

	/**
	 * This method instantiates the new thread for the PID loop will run in.
	 * Although the PID loop thread has been created, the PID loop will not start
	 * running until a call to {@link #Enable() Enable()} method has been made.
	 */
	public void startThread() {
		this.executor = new java.util.Timer();
		this.executor.schedule(new PIDSpeedTask(this), 0L, this.period);

	}

	/**
	 * This method enables the PID Loop calculation. After this method is called the
	 * controller will calculate the control output in its own thread once every
	 * period of the loop defined by the parameter passed to the constructor. This
	 * method should only be called after a call to @link {@link #startThread()} has
	 * been called first.
	 */
	public void Enable() {

		this.enable = true;
	}

	/**
	 * This method disables the PID loop calculation. Call this method when ever the
	 * PID loop is not needed to help reduce CPU utilization. This method does not
	 * kill the thread so a simple call to @link #Enable() will allow the
	 * calculations to start again.
	 * 
	 */
	public void Pause() {

		// disable PID loop
		this.enable = false;

		this.isFinished = false;

		reset();

	}

	/**
	 * This method is for debugging purposes only. There is no need to call this
	 * method during a competition. This method will reset all parameters back to
	 * its default. NOTE: Do not call this method when the loop is running. Damage
	 * to your system could result from the sudden change in control variables.
	 */
	public void reset() {
		this.isFinished = false;

		// zero all other parameters
		// this.acceptErrorDiff=0;
		this.clock = 0;
		this.co = 0;
		this.coNotSaturated = 0;
		this.coOld = 0;
		this.cp = 0;
		this.prop = 0;
		this.deriv = 0;
		this.integ = 0;
		this.err = 0;
		this.errsum = 0;
		this.filterDerivOld = 0;
		this.olderr = 0;
		this.olderrsum = 0;
	}

	/**
	 * @return the current Proportional Gain in type double
	 */
	public double getPGain() {
		return pGain;
	}

	/**
	 * @return the current Integral Gain in type double
	 */
	public double getIGain() {
		return iGain;
	}

	/**
	 * @return the current Integral Gain in type double
	 */
	public double getDGain() {
		return dGain;
	}

	/**
	 * @return If Gain Scheduling is enabled this will return the Proportional Gain
	 *         to be used when the error is less than zero. If Gain Scheduling is
	 *         not enabled, this will return the same value as @link
	 *         {@link #getPGain()}
	 */
	public double getPGain2() {
		return pGain2;
	}

	/**
	 * @return If Gain Scheduling is enabled this will return the Integral Gain to
	 *         be used when the error is less than zero. If Gain Scheduling is not
	 *         enabled, this will return the same value as @link {@link #getIGain()}
	 */
	public double getIGain2() {
		return iGain2;
	}

	/**
	 * @return If Gain Scheduling is enabled this will return the Derivative Gain to
	 *         be used when the error is less than zero. If Gain Scheduling is not
	 *         enabled, this will return the same value as @link {@link #getDGain()}
	 */
	public double getDGain2() {
		return dGain2;
	}

	/**
	 * @return the boolean flag indicating if gain Scheduling is enabled. Gain
	 *         scheduling is enabled when true, if false gain scheduling is not
	 *         enabled. Gain Scheduling is set to false by default.
	 */
	public boolean isEnGainSched() {
		return enGainSched;
	}

	/**
	 * @return the boolean flag indicating if the PID loop is enabled. The PID is
	 *         enabled when true, if false the PID Loop has not been enabled. Enable
	 *         is set to false by default.
	 */
	public boolean isEnabled() {
		return enable;
	}

	/**
	 * @return the boolean flag indicating if Derivative filtering is enabled.
	 *         Derivative filtering is enabled when true, if false Derivative
	 *         filtering is not enabled. Derivative filtering is set to false by
	 *         default.
	 */
	public boolean isDerivFilterEnabled() {
		return enDerivFilter;
	}

	/**
	 * @return the current Error between the current velocity and the setPoint
	 *         velocity in type double
	 */
	public double getError() {
		return err;
	}

	/**
	 * @return the current setPoint the PID controller is to achieve in type double
	 */
	public double getSetPoint() {
		return sp;
	}

	/**
	 * @return the rate output of the Speed Sensor used with this PID Controller in
	 *         type double. The unit of this value is in the native unit of the
	 *         Speed Sensor.
	 */
	public double getSensorRate() {
		return encoder.getRate();
	}

	/**
	 * @return The controller output value. This is the output to use to drive the
	 *         motors based on the PID control. It is saturated to the minimum and
	 *         maximum values set by @link {@link #setMinPosOutput(double)},@link
	 *         {@link #setMinPosOutput(double)},@link
	 *         {@link #setMaxNegOutput(double)}, @link
	 *         {@link #setMinNegOutput(double)} so it does not drive the PWM to the
	 *         motor controller beyond its limit. For FRC the @link
	 *         {@link #setMaxPosOutput(double)} and @link
	 *         {@link #setMaxNegOutput(double)} should not be more that 1.
	 */
	public double getControlOutput() {
		return co;
	}

	/**
	 * @return This is for debugging purposed only and should not be used to drive
	 *         an actual device. This returns the raw controller output before any
	 *         filtering is done. The values returned may very well be higher than
	 *         the values your device can handle and can command it to dangerous
	 *         levels. Use this when graphing the output to see where the @link
	 *         {@link #getControlOutput()}. This is for debugging purposes only.
	 *         <br>
	 *         <br>
	 * 
	 *         Use the @link {@link #getControlOutput()} method to drive a device
	 *         based on controller output.
	 */
	public double getCoNotSaturated() {
		return coNotSaturated;
	}

	/**
	 * @return the Returns the Actual Time between loops. Use this value to verify
	 *         your loop is running at the proper rate specified by the Period
	 *         provided in the Constructor.
	 */
	public double getExecutionTime() {
		return executionTime;
	}

	/**
	 * @return the Derivative filter employs a Euler Filter to cancel unwanted
	 *         Derivative Noise. This function returns the weighting factor used in
	 *         the Euler filter such that if the weight is (1-r), this method
	 *         returns the current r value set.
	 */
	public double getDerivativeFilterConstant() {
		return r;
	}

	/**
	 * @return the current maximum Positive Output the Controller can output
	 *         using @link {@link #getControlOutput()}. This is useful to limit the
	 *         Conrol Output to a range which your actuator or driving circuit can
	 *         tolerate.
	 */
	public double getMaxPosOutput() {
		return maxPosOutput;
	}

	/**
	 * @return the current maximum Negative Output the Controller can output
	 *         using @link {@link #getControlOutput()}. This is useful to limit the
	 *         Control Output to a range which your actuator or driving circuit can
	 *         tolerate.
	 */
	public double getMaxNegOutput() {
		return maxNegOutput;
	}

	/**
	 * @return the current minimum Positive Output the Controller can output
	 *         using @link {@link #getControlOutput()}. This is useful to limit the
	 *         Conrol Output to a range which your actuator or driving circuit can
	 *         tolerate.
	 */
	public double getMinPosOutput() {
		return minPosOutput;
	}

	/**
	 * @return the current minimum Negative Output the Controller can output
	 *         using @link {@link #getControlOutput()}. This is useful to limit the
	 *         Control Output to a range which your actuator or driving circuit can
	 *         tolerate.
	 */
	public double getMinNegOutput() {
		return minNegOutput;
	}

	/**
	 * @return the acceptable error different between the setPoint and current rate
	 *         of the system
	 */
	public double getAcceptErrorDiff() {
		return acceptErrorDiff;
	}

	/**
	 * @return the period of the thread this PID loop is running in.
	 */
	public double getPeriod() {
		return period;
	}

	/**
	 * @param pGain
	 *            Sets the controllers default Proportional gain to the value of
	 *            pGain. Use this method when changing gains during tuning. Value
	 *            may take one period to be read by the PID loop. If Gain Scheduling
	 *            is enabled, pGain will only be used when the error is positive.
	 */
	public void setpGain(double pGain) {
		this.pGain = pGain;
	}

	/**
	 * @param iGain
	 *            Sets the controllers default Integral gain to the value of iGain.
	 *            Use this method when changing gains during tuning. Value may take
	 *            one period to be read by the PID loop. If Gain Scheduling is
	 *            enabled, iGain will only be used when the error is positive.
	 */
	public void setiGain(double iGain) {
		this.iGain = iGain;
	}

	/**
	 * @param dGain
	 *            Sets the controllers default Derivative gain to the value of
	 *            iGain. Use this method when changing gains during tuning. Value
	 *            may take one period to be read by the PID loop. If Gain Scheduling
	 *            is enabled, dGain will only be used when the error is positive.
	 */
	public void setdGain(double dGain) {
		this.dGain = dGain;
	}

	/**
	 * @param pGain2
	 *            Used when Gain Scheduling is enabled. To enable Gain Scheduling
	 *            see @link {@link #enGainSched}. This method sets the Proportional
	 *            gain of the Controller to pGain2. This value is only used when the
	 *            error goes negative. Use this method when changing gains during
	 *            tuning. Value may take one period to be read by the PID loop.
	 */
	public void setpGain2(double pGain2) {
		this.pGain2 = pGain2;
	}

	/**
	 * @param iGain2
	 *            Used when Gain Scheduling is enabled. To enable Gain Scheduling
	 *            see @link {@link #enGainSched}. This method sets the Integral gain
	 *            of the Controller to iGain2. This value is only used when the
	 *            error goes negative. Use this method when changing gains during
	 *            tuning. Value may take one period to be read by the PID loop.
	 */
	public void setiGain2(double iGain2) {
		this.iGain2 = iGain2;
	}

	/**
	 * @param dGain2
	 *            Used when Gain Scheduling is enabled. To enable Gain Scheduling
	 *            see {@link #enGainSched}. This method sets the Derivative gain of
	 *            the Controller to dGain2. This value is only used when the error
	 *            goes negative. Use this method when changing gains during tuning.
	 *            Value may take one period to be read by the PID loop.
	 */
	public void setdGain2(double dGain2) {
		this.dGain2 = dGain2;
	}

	/**
	 * @param enGainSched
	 *            This method allows the user to enable or disable PID Gain
	 *            Scheduling. Gain Scheduling is useful for situations where
	 *            different PID gains are to be used when the error between the
	 *            setpoint and the current system is Positive vs Negative. For
	 *            example yo may way to have different gains for forward and reverse
	 *            on a drivetrain, or if you were controlling an Arm, you may
	 *            different gains for up and down. TRUE will enable gain scheduling,
	 *            FALSE will disable gain scheduling. It may take one period cycle
	 *            for this to take affect in the control loop. Use
	 *            {@link #setpGain(double)}, {@link #setiGain(double)},
	 *            {@link #setdGain(double)}, {@link #setpGain2(double)},
	 *            {@link #setiGain2(double)}, {@link #setdGain2(double)}
	 */
	public void setEnGainSched(boolean enGainSched) {
		this.enGainSched = enGainSched;
	}

	/**
	 * @param enDerivFilter
	 *            This method allows the user to enable or disable Derivative
	 *            Filtering. The derivative term can induce noise into a PID
	 *            controller rearing it self as oscillations in the output of the
	 *            controller. Filtering the derivative term can reduce this effect.
	 *            TRUE will enable derivative filtering, FALSE will disable
	 *            derivative filtering. It may take one period cycle for this to
	 *            take affect in the control loop. Use
	 */
	public void setEnDerivFilter(boolean enDerivFilter) {
		this.enDerivFilter = enDerivFilter;
	}

	/**
	 * @param sp
	 *            This method sets the setpoint the controller is to achieve. Use
	 *            this method to change the setpoint, if the controller is enabled
	 *            it will automatically start to achieve the new setpoint.
	 */
	public void setSetPoint(double sp) {
		this.setPointByArray = false;
		this.sp = sp;
	}


	/**
	 * @param sp
	 *            This method sets the setpoint the controller is to achieve. Use
	 *            this method to change the setpoint, if the controller is enabled
	 *            it will automatically start to achieve the new setpoint.
	 */
	public void setSetPoint(double[] sp) {
		this.setPointByArray = true;

		// copy array
		this.setPointArray = new double[sp.length];
		for (int i = 0; i < setPointArray.length; i++)
			setPointArray[i] = sp[i];

		this.setPointArrayCounter = 0;

	}
	
	/**
	 * @param sp
	 *            This method sets the setpoint the controller is to achieve. Use
	 *            this method to change the setpoint, if the controller is enabled
	 *            it will automatically start to achieve the new setpoint.
	 */
	public void setSetPoint(double[][] sp) {
		this.setPointByArray = true;

		// copy array
		this.setPointArray = new double[sp.length];
		for (int i = 0; i < setPointArray.length; i++)
			setPointArray[i] = sp[i][1];

		this.setPointArrayCounter = 0;

	}

	/**
	 * @param r
	 *            the derivative filter uses an Euler filter to filter the
	 *            derivative. The weighting of the filter is (1-r). This method will
	 *            set the value of r.
	 */
	public void setDerivFilterGain(double r) {
		this.r = r;
	}

	/**
	 * @param maxPosOutput
	 *            the maximum Positive Output the control output can take. This is
	 *            useful to clamp the output within the range of the motor being
	 *            controlled.
	 */
	public void setMaxPosOutput(double maxPosOutput) {
		this.maxPosOutput = maxPosOutput;
	}

	/**
	 * @param maxNegOutput
	 *            the maximum Negative Output the control output can take. This is
	 *            useful to clamp the output within the range of the motor being
	 *            controlled.
	 */
	public void setMaxNegOutput(double maxNegOutput) {
		this.maxNegOutput = maxNegOutput;
	}

	/**
	 * @param minPosOutput
	 *            the minimum Positive Output the control output can take. This is
	 *            useful to clamp the output within the range of the motor being
	 *            controlled.
	 */
	public void setMinPosOutput(double minPosOutput) {
		this.minPosOutput = minPosOutput;
	}

	/**
	 * @param minNegOutput
	 *            the minimum Negative Output the control output can take. This is
	 *            useful to clamp the output within the range of the motor being
	 *            controlled.
	 */
	public void setMinNegOutput(double minNegOutput) {
		this.minNegOutput = minNegOutput;
	}

	/**
	 * @param acceptErrorDiff
	 *            this is the offset you are willing to tolerate between the
	 *            setPoint and the plant. This should not be zero or the controller
	 *            will never come to rest because true zero is never reachable.
	 *            Always make sure there is some tolerance. With this value set, the
	 *            controller will stop when it is within +/- this value.
	 */
	public void setAcceptErrorDiff(double acceptErrorDiff) {
		this.acceptErrorDiff = acceptErrorDiff;
	}

	public int getSIZE() {
		return SIZE;
	}

	/**
	 * 
	 * @param size
	 *            size of the array used to average the output. When the array
	 * 
	 */
	public void setSIZE(int size) {
		this.SIZE = size;
		this.atSpeed = new double[SIZE];
	}

	/**
	 * @return the name of the controller. This is useful when debugging multiple
	 *         loops and keeping track of each controller on your system.
	 */
	public String getName() {
		return name;
	}

	/**
	 * @param name
	 *            the name to change the controller too. This is useful when
	 *            debugging multiple loops and keeping track of each controller on
	 *            your system.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * 
	 * @return this boolean returns true if the controller has reached the setpoint
	 *         and false otherwise. The controller uses an array and stores the
	 *         mutiple previous values of the system to ensure that it is truly at
	 *         the setpoint before reporting true.
	 */
	public boolean isFinished() {

		return this.isFinished;
	}

	/**
	 * returns true if current sp is controlled by an array, false otherwise
	 * 
	 * 
	 */
	public boolean isSetPointByArray() {

		return this.setPointByArray;
	}

	/**
	 * Function to see if the wheel is at steady state speed
	 */
	private void atSpeed() {
		// // finish is based on verifying the voltage is constant over some length
		// // of time
		// if (count == this.atSpeed.length)
		// count = 0;
		//
		// atSpeed[count] = co;
		// count++;
		//
		// int inRange = 0;
		// for (int j = 1; j < atSpeed.length; j++) {
		// if ((this.cp < this.sp + 25) && (this.cp > this.sp - 25))
		// inRange++;
		// else
		// inRange = 0;
		// }
		// if (inRange == atSpeed.length - 1)
		// isFinished = true;
		// else {
		// isFinished = false;
		// }

		if (this.cp >= this.sp - 30)
			isFinished = true;
		else
			isFinished = false;

	}

	/**
	 * This is used with a TCP Stream debugger to show the values of the PID loop.
	 */

	public synchronized String sendJSON() {

		return "{" + "\"_expected Period\":" + this.period + "," + "\"_execcution Time\":" + this.runTime + ","
				+ "\"_output\":" + this.co + "," + "\"_error\":" + this.err + "," + "\"_Prop Term\":" + this.prop + ","
				+ "\"_Integ Term\":" + this.integ + "," + "\"_Deriv Term\":" + this.deriv + "," + "\"_Error Sum\":"
				+ this.errsum + "," + "\"_CO Unsaturated\":" + this.coNotSaturated + "," + "\"_P_Used\":" + this.p + ","
				+ "\"_I_Used\":" + this.i + "," + "\"_D_Used\":" + this.d + "," +

				"\"_Encoder Rate\":" + encoder.getRate() + "," + "\"_setPoint\":" + this.sp + "," +

				"\"_max Pos Output\":" + this.maxPosOutput + "," + "\"_max Neg Output\":" + this.maxNegOutput + ","
				+ "\"_min Pos Output\":" + this.minPosOutput + "," + "\"_min Neg Output\":" + this.minNegOutput + "," +

				"\"_deriv Filter Constant\":" + this.r + "," + "\"_acceptable Err\":" + this.acceptErrorDiff + "," +

				// boolean dashboard
				"\"_PID Enabled\":" + this.enable + "," + "\"_debug Enabled\":" + this.enable + ","
				+ "\"_deriv Filter Enabled\":" + this.enDerivFilter + "," + "\"_Gain Sched Enabled\":"
				+ this.enGainSched + "," + "\"_is Finished\":" + this.isFinished + "}\n";
	}

	public synchronized String JSONInit() {
		return "{" + "\"_P_Used_init\":" + this.pGain + "," + "\"_I_Used_init\":" + this.iGain + ","
				+ "\"_D_Used_init\":" + this.dGain + "," + "\"_setPoint_init\":" + this.sp + "," +

				"\"_max Pos Output_init\":" + this.maxPosOutput + "," + "\"_max Neg Output_init\":" + this.maxNegOutput
				+ "," + "\"_min Pos Output_init\":" + this.minPosOutput + "," + "\"_min Neg Output_init\":"
				+ this.minNegOutput + "," +

				"\"_deriv Filter Constant_init\":" + this.n + "," + "\"_acceptError_init\":" + this.acceptErrorDiff
				+ "," + "\"_array_size_init\":" + this.SIZE + "," + "\"_name\":" + "\"" + this.name + "\"" +

				"}\n";
	}

	/**
	 * This is used with a TCP stream debugger to change values of this PId
	 * controller on the fly
	 */

	public synchronized void receiveJSON(String[] message) {
		// System.out.println("receive command");
		try {
			this.pGain = Double.valueOf(message[0]).doubleValue();
			this.iGain = Double.valueOf(message[1]).doubleValue();
			this.dGain = Double.valueOf(message[2]).doubleValue();
			this.sp = Double.valueOf(message[3]).doubleValue();
			this.maxPosOutput = Double.valueOf(message[5]).doubleValue();
			this.maxNegOutput = Double.valueOf(message[6]).doubleValue();
			this.minPosOutput = Double.valueOf(message[7]).doubleValue();
			this.minNegOutput = Double.valueOf(message[8]).doubleValue();
			// this.enDerivFilter = TCPsocketSender.strToBool(message[9]);
			this.acceptErrorDiff = Double.valueOf(message[10]).doubleValue();
			// setSIZE(Integer.valueOf(message[11]).intValue());

			// if(Boolean.valueOf(message[4]).booleanValue())
			// new DriveRightPIDSpeed(Double.valueOf(message[3]).doubleValue()).start();
			// else
			// new DrivePIDPause().start();

		} catch (NumberFormatException e) {
			// System.out.println("Don't send empty values");
		}

		// $('#P_Gain').val() +','+
		//
		// $('#I_Gain').val() +','+
		//
		// $('#D_Gain').val() +','+
		//
		// $('#setSetPt').val() +','+
		//
		// $('#PID_Ena'.is(':checked') +','+
		//
		// $('#MxPOutTR').val() +','+
		//
		// $('#MxNOutTR').val() +','+
		//
		// $('#MnPOutTR').val() +','+
		//
		// $('#MnNOutTR').val() +','+
		//
		// $('#DervFltEn'.is(':checked') +','+
		//
		// $('#DrvFltrCns').val() +','+
		//
		// $('#acceptErr').val() +','+
		//
		// $('#Array Size').val() +','+

		// copy values
	}

	/**
	 * Method runs a PID loop. This method should not be called directly. This
	 * method should only be called from a PIDSpeedTask object which runs this
	 * method in a periodic thread.
	 */
	private synchronized void calculate() {
		runTime = Timer.getFPGATimestamp();

		if (enable) {
			// poll encoder
			if (encoder == null)
				throw new NullPointerException(" Feedback Encoder was null");

			// noticed that WPILibJ Encoder will sometimes throw a NaN for
			// getRate. Although
			// This could potentially NaN all values of the Control Loop for the
			// period which
			// the encoder returns NaN. We check for the Nan Value and clamp the
			// rate to its last
			// value.
			double tempRate = encoder.getRate();
			if (!Double.isNaN(tempRate))
				cp = tempRate; // cp is in nominal units returned by the sensor

			// allow setPoint to be updated by array
			if (setPointByArray == true && setPointArrayCounter < setPointArray.length) {
				sp = setPointArray[setPointArrayCounter];
				setPointArrayCounter++;
			} else
				setPointByArray = false;

			// if gain schedule has been enabled, make sure we use
			// proper PID gains
			if (enGainSched && err < 0) {
				p = pGain2;
				i = iGain2;
				d = dGain2;
			} else {
				p = pGain;
				i = iGain;
				d = dGain;
			}

			// calculate error between current position and setpoint
			err = sp - cp;

			// calculate derivative gain d/dt
			double currentTime = Timer.getFPGATimestamp();
			executionTime = currentTime - clock; // time

			// integral
			boolean windup = false;
			errsum = errsum + (olderr * executionTime);
			integ = i * errsum; // final integral term

			deriv = 0;

			// // deriv term
			// if (enDerivFilter) {
			// 	// Derivative filtering using forward euler integration
			// 	int_d_term = int_d_term + (lastDeriv * executionTime);
			// 	deriv = ((d * err) - int_d_term) * n;
			// 	lastDeriv = deriv;
			// } else {
				// prevent divide by zero error, by disabiling deriv term
				// if execution time is zero.
				diff = 0;
				if (executionTime > 0)
					diff = (err - olderr) / executionTime; // delta
				else
					diff = 0;

				deriv = d * diff;
			// }

			// proportional term
			prop = p * err;

			// calculate new control output based on filtering
			co = prop + integ + deriv;

			// integral anti-windup control via clamping
			// essentially assume error is zero in this case
			if ((co > maxPosOutput || co < maxNegOutput) && (Math.signum(co) == Math.signum(i * err))) {
				errsum = errsum - (olderr * executionTime);
				integ = i * errsum;
				co = prop + integ + deriv;
				windup = true;
			}

			// save control output for graphing
			coNotSaturated = co;

			// deadband compensation
			if (co < minPosOutput && co >= 0) // controller in pos deadband
			{
				errsum = (minPosOutput / i) - prop - deriv;
				integ = errsum * i;
				co = prop + deriv + integ;
			} else if (co > minNegOutput && co <= 0) // controller in neg deadband)
			{
				errsum = (minNegOutput / i) - prop - deriv;
				integ = errsum * i;
				co = prop + deriv + integ;
			}

			// Saturation
			if (co > maxPosOutput)
				co = maxPosOutput;
			if (co < maxNegOutput)
				co = maxNegOutput;

			coOld = co;

			// FIXME : Make apart of method
			if (Math.abs(err) < acceptErrorDiff) {
				co = coOld;
				// this.isFinished = true;
			} else {
				// this.isFinished = false;
			}
			// update clock with current time for next loop
			clock = currentTime;
			olderr = err;

			// System.out.println("time: " + currentTime + "\tcperr: " + cp + "\tsp: " + sp
			// + "\terr: " + err + "\tpterm: " + prop + "\twindup: " + windup + "\terrsum: "
			// + errsum +"\titerm: " + integ + "\tdterm: " + deriv + "\toutput" + co +
			// "\texctime" + executionTime );
			if(log != null)
			{
			log.println(currentTime + "\t" + System.currentTimeMillis() + "\t" + cp + "\t" + sp + "\t " + err + "\t"
					+ prop + "\t" + windup + "\t" + errsum + "\t" + integ + "\t" + deriv + "\t" + co + "\t"
					+ coNotSaturated + "\t" + executionTime);
			log.flush();
			}

			atSpeed();
		} else {
			cp = encoder.getRate();
			sp = encoder.getRate();
			clock = Timer.getFPGATimestamp();
			isFinished = false;
		}

		runTime = Timer.getFPGATimestamp() - runTime;
	}

	/**
	 * Private internal class which spawns a new thread for every PID object
	 * 
	 * @author HarrilalEngineering
	 * 
	 */
	private class PIDSpeedTask extends TimerTask {
		// internal PIDSpeed object to run in new thread
		private PIDSpeed speedController;

		/**
		 * constructor for the private class PIDSpeedTask, which will be used to spawn a
		 * new thread. Each thread will continuously run the run() function
		 * 
		 * @param id
		 *            a string used to identify this particular controller
		 * @param controller
		 *            the controller parameters used to create the thread
		 */
		private PIDSpeedTask(PIDSpeed controller) {

			if (controller == null) {
				throw new NullPointerException(" PIDController was null");
			}
			speedController = controller;
		}

		/**
		 * called periodically in its own thread
		 */

		public void run() {
			speedController.calculate();

		}
	}

}