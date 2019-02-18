//package org.team2168.utils;
//
//import java.util.TimerTask;
//
//import org.team2168;
//import org.team2168Map;
//import org.team2168.utils.consoleprinter.ConsolePrinter;
//
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
//
////import org.team2168Map;
//
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
//import edu.wpi.first.wpilibj.Timer;
//
//public class TX1TurnON {
//	private java.util.Timer executor;
//	private long period;
//
//	public static DigitalOutput tx1TurnOn;
//	public static DigitalInput tx1OnStatus;
//
//	public TX1TurnON(long period) {
//		this.period = period;
//
//		tx1TurnOn = new DigitalOutput(RobotMap.TX1_TURN_ON);
//		tx1OnStatus = new DigitalInput(RobotMap.TX1_ON_STATUS);
//
//		ConsolePrinter.putBoolean("TX1TurnOn", () -> {
//			return getTX1TurnOn();
//		}, true, false);
//		ConsolePrinter.putBoolean("TX1OnStatus", () -> {
//			return getTX1OnStatus();
//		}, true, false);
//	}
//
//	public void startThread() {
//		this.executor = new java.util.Timer();
//		this.executor.schedule(new TX1TurnOnTask(this), 0L, this.period);
//
//	}
//
//	public boolean getTX1TurnOn() {
//		return !tx1TurnOn.get();
//	}
//
//	/**
//	 * Returns status of TX1 on. This DIO input pin should be plugged into GND on
//	 * TX1 J17 header
//	 */
//	public boolean getTX1OnStatus() {
//		return !tx1OnStatus.get();
//	}
//
//	/**
//	 * Try to press tegra TX1 button 1 every other second This DIO output pin should
//	 * be plugged into the PNL pin of the J6 header on TX1
//	 */
//	public void turnTX1On() {
//		if (Math.ceil(Timer.getFPGATimestamp()) % 2 == 0) {
//			tx1TurnOn.set(false);
//		} else {
//			tx1TurnOn.set(true);
//		}
//	}
//
//	private void run() {
//
//		turnTX1On();
//
//	}
//
//	private class TX1TurnOnTask extends TimerTask {
//		private TX1TurnON tx1;
//
//		private TX1TurnOnTask(TX1TurnON tx1) {
//			if (tx1 == null) {
//				throw new NullPointerException("TX1 was null");
//			}
//			this.tx1 = tx1;
//		}
//
//		/**
//		 * Called periodically in its own thread
//		 */
//		public void run() {
//			tx1.run();
//		}
//	}
//
//}
