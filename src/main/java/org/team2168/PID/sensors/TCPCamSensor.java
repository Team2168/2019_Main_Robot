package org.team2168.PID.sensors;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

import org.team2168.utils.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The TCPCameraSensor class is used to grab data from a TCP socket and provide
 * it for use on an FRC robot. The intended use of this class is to retrieve
 * data from a tegra running Vision. This class uses 3 threads, one which
 * listens for incoming connections, a second which retrieves data from the
 * tegra, and a thrid which can be used to send data to the tegra.
 * 
 * @author HarrilalEngineering
 */

public class TCPCamSensor implements PIDSensorInterface {

	public boolean isCloseShot = false;

	private int port;
	private String messageOut;
	private byte[] buf;
	private volatile String[] dataReceived;
	private StringBuffer sb = new StringBuffer();
	private volatile boolean sendEnable;

	private volatile boolean clientConnected;

	private DriverStation ds;

	// A TCP Socket Connection
	private ServerSocket conn = null;

	// TCP Socket Stream
	private Socket sc = null;

	// Threads
	private Thread t1;
	private Thread t2;
	private Thread t3;

	private long requestPeriod;

	private int size;

	private String name;

	// Default HSV for OpenCV
	private int hMin = 0;
	private int hMax = 180;
	private int sMin = 0;
	private int sMax = 255;
	private int vMin = 0;
	private int vMax = 255;

	/**
	 * 
	 * @param port
	 *            which is to be used to Listen for incoming TCP connections on the
	 *            FRC bot.
	 */
	public TCPCamSensor(String name, int port, long requestPeriod) {
		this.requestPeriod = requestPeriod;

		this.name = name;

		size = 15;

		// initialize data messageOut
		dataReceived = new String[size];

		dataReceived[0] = "0";
		dataReceived[1] = "0";
		dataReceived[2] = "0";
		dataReceived[3] = "0";
		dataReceived[4] = "0";
		dataReceived[5] = "0";
		dataReceived[6] = "0";
		dataReceived[7] = "0";
		dataReceived[8] = "0";
		dataReceived[9] = "0";
		dataReceived[10] = "0";
		dataReceived[11] = "0";
		dataReceived[12] = "0";
		dataReceived[13] = "0";
		dataReceived[14] = "0";

		// setup socket to listen on
		this.port = port;

		ds = DriverStation.getInstance();

		SmartDashboard.putNumber(getName() + "_MinH_current", getHueMin());
		SmartDashboard.putNumber(getName() + "_MaxH_current", getHueMax());
		SmartDashboard.putNumber(getName() + "_MinS_current", getSaturationMin());
		SmartDashboard.putNumber(getName() + "_MinS_current", getSaturationMax());
		SmartDashboard.putNumber(getName() + "_MinV_current", getValueMin());
		SmartDashboard.putNumber(getName() + "_MaxV_current", getValueMax());

		start();

	}

	public void start() {
		t3 = new Thread(new Runnable() {

			public void run() {

				try {

					// Opens a socket to listen for incoming connections
					try {
						conn = new ServerSocket(port);

					} catch (IOException e) {
						e.printStackTrace();
					}

					// wait for a client to connect, this blocks until a connect
					// is made
					clientConnected = false;
					System.out.println(
							"Listening on: " + conn.getLocalSocketAddress() + " on port: " + conn.getLocalPort());
					sc = conn.accept();
					System.out.println("Client Connected");
					clientConnected = true;

					// make this true if you want to send data to the tegra
					// as well
					sendEnable = true;

					listener();
					sender();

				} catch (IOException e) {

				}
			}
		}

		);

		t3.start();
	}

	private void listener() {

		t1 = new Thread(new Runnable() {

			public void run() {
				try {
					InputStream is = null;
					is = sc.getInputStream();
					int ch = 0;
					int count = 0;

					// read data until newline character is reached
					while ((ch = is.read()) != -1) {
						if ((char) ch != '\n')
							sb.append((char) ch);
						else {
							// print data received to the screen
							// split data into array
							dataReceived = Util.split(sb.toString(), ","); // splits
							System.out.println(Arrays.toString(dataReceived));
							System.out.println("Match Start: " + isMatchStart() + ", " + "Target Rotation: "
									+ getRotationAngle() + ", " + "Target Distance: " + getTargetDistance());
							System.out.flush();

							// set HSVs with default values
							if (count == 0) {
								SmartDashboard.putNumber(getName() + "_MinH_set", getHueMin());
								SmartDashboard.putNumber(getName() + "_MaxH_set", getHueMax());
								SmartDashboard.putNumber(getName() + "_MinS_set", getSaturationMin());
								SmartDashboard.putNumber(getName() + "_MaxS_set", getSaturationMax());
								SmartDashboard.putNumber(getName() + "_MinV_set", getValueMin());
								SmartDashboard.putNumber(getName() + "_MaxV_set", getValueMax());
								count++;
							}

							SmartDashboard.putNumber(getName() + "_MinH_current", getHueMin());
							SmartDashboard.putNumber(getName() + "_MaxH_current", getHueMax());
							SmartDashboard.putNumber(getName() + "_MinS_current", getSaturationMin());
							SmartDashboard.putNumber(getName() + "_MaxS_current", getSaturationMax());
							SmartDashboard.putNumber(getName() + "_MinV_current", getValueMin());
							SmartDashboard.putNumber(getName() + "_MaxV_current", getValueMax());
							// create new buffer
							sb = new StringBuffer();
						}
					}
				} catch (IOException x) {
					x.printStackTrace();
				}
			}

		}

		);

		t1.start();
	}

	private void sender() {
		t2 = new Thread(new Runnable() {

			@SuppressWarnings("deprecation")
			public void run() {
				OutputStream os = null;
				int count = 0;
				try {
					os = sc.getOutputStream();

					while (sendEnable) {
						// we want to send if match has started to camera
						int matchStart = 0;

						if (ds.isEnabled())
							matchStart = 1;

						messageOut = String.valueOf(matchStart) + " " + count + " "
								+ SmartDashboard.getNumber(getName() + "_MinH_set", getHueMin()) + " "
								+ SmartDashboard.getNumber(getName() + "_MaxH_set", getHueMax()) + " "
								+ SmartDashboard.getNumber(getName() + "_MinS_set", getSaturationMin()) + " "
								+ SmartDashboard.getNumber(getName() + "_MaxS_set", getSaturationMax()) + " "
								+ SmartDashboard.getNumber(getName() + "_MinV_set", getValueMin()) + " "
								+ SmartDashboard.getNumber(getName() + "_MaxV_set", getValueMax()) + " \n";

						System.out.println("Sending Match Start: " + messageOut);
						System.out.flush();
						buf = messageOut.getBytes(StandardCharsets.US_ASCII);

						count++;

						try {
							os.write(buf);
						} catch (IOException e) {
							// e.printStackTrace();
							System.out.println("Appears Client Closed " + "the Connection");
							System.out.flush();
							stopThreads();

							// close streams
							os.close();
							sc.close();
							conn.close();

							// restart server
							start();

						}

						try {
							Thread.sleep(requestPeriod);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}

				} catch (IOException e) {
					e.printStackTrace();
				}

			}
		}

		);
		t2.start();
	}

	private void stopThreads() {
		sendEnable = false;
	}

	public int getMessageLength() {

		return size;
	}

	public String[] getMessage() {

		return dataReceived;
	}

	// These are specific to the game, modify for each year
	public boolean isMatchStart() {

		int message = Integer.valueOf(dataReceived[0]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public double getRotationAngle() {

		if (dataReceived[1].compareToIgnoreCase("inf") != 0 && dataReceived[1].compareToIgnoreCase("-inf") != 0
				&& dataReceived[1].compareToIgnoreCase("+inf") != 0) {
			double message = Double.valueOf(dataReceived[1]).doubleValue();
			return message;
		} else
			return 0;
	}

	public double getTargetDistance() {

		if (dataReceived[2].compareToIgnoreCase("inf") != 0 && dataReceived[2].compareToIgnoreCase("-inf") != 0
				&& dataReceived[2].compareToIgnoreCase("+inf") != 0) {
			double message = Double.valueOf(dataReceived[2]).doubleValue();
			return message;
		} else
			return 0;

	}

	public boolean isClientConnected() {
		return clientConnected;

	}

	public boolean isTargetScorable() {
		int message = Integer.valueOf(dataReceived[4]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public boolean isProcessingTreadRunning() {
		int message = Integer.valueOf(dataReceived[5]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public boolean isCameraConnected() {
		int message = Integer.valueOf(dataReceived[6]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public boolean isMJPEGConnected() {

		int message = Integer.valueOf(dataReceived[7]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public boolean isTargetDetected() {
		int message = Integer.valueOf(dataReceived[6]).intValue();

		if (message == 1)
			return true;
		else
			return false;

	}

	public int getHueMin() {

		int message = Integer.valueOf(dataReceived[9]).intValue();
		return message;
	}

	public int getHueMax() {

		int message = Integer.valueOf(dataReceived[10]).intValue();
		return message;
	}

	public int getSaturationMin() {

		int message = Integer.valueOf(dataReceived[11]).intValue();
		return message;
	}

	public int getSaturationMax() {

		int message = Integer.valueOf(dataReceived[12]).intValue();
		return message;
	}

	public int getValueMin() {

		int message = Integer.valueOf(dataReceived[13]).intValue();
		return message;
	}

	public int getValueMax() {

		int message = Integer.valueOf(dataReceived[14]).intValue();
		return message;
	}

	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub

	}

	@Override
	public double getPos() {
		// TODO Auto-generated method stub
		return getRotationAngle();
	}

	public String getName() {
		return this.name;
	}

}
