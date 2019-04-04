package org.team2168.utils;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class TCPSocketSender {
	int port = 4444;
	int portNumber = 41234;
	String message;
	byte[] buf;
	String[] arr;
	StringBuffer sb = new StringBuffer();
	boolean enable;
	boolean init;

	private volatile boolean clientConnected;

	// A datagram connection.
	private ServerSocket listener;
	// A datagram which holds sent and received data.

	// The address of the Network Time Protocol (NTP) daemon
	// process on a particular server. NTP uses the UDP
	// protocol.
	private String addressIn = "socket://:1180";

	private Object lock1 = new Object();
	private Object lock2 = new Object();
	// Create a socket to listen on the port.

	Thread t1;
	Thread t2;
	Thread t3;

	private Socket client = null;

	private TCPMessageInterface JSON;

	public TCPSocketSender(int port, TCPMessageInterface obj) {
		arr = new String[3];
		arr[0] = "0";
		arr[1] = "0";
		arr[2] = "0";

		if (obj != null)
			JSON = obj;
		// Open a client connection.

		addressIn = "socket://:" + port;

		enable = false;
		init = false;

		// sc = (SocketConnection) Connector.open(addressOut);

		try {
			listener = new ServerSocket(port);

			System.out.println(
					"Listening on: " + listener.getLocalSocketAddress() + " on port: " + listener.getLocalPort());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void start() {
		t3 = new Thread(new Runnable() {

			public void run() {

				try {

					clientConnected = false;
					System.out.println("Listening on port: " + listener.getLocalPort());

					client = listener.accept();

					System.out.println("Client Connected");
					clientConnected = true;

					listener();
					sender();

				} catch (IOException e) {
					e.printStackTrace();

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
					DataInputStream is = new DataInputStream(client.getInputStream());
					int ch = 0;

					while ((ch = is.read()) != -1) {

						if ((char) ch != '\n')
							sb.append((char) ch);
						else {
							System.out.println(client.getLocalPort()+": "+sb.toString());

							arr = Util.split(sb.toString(), ","); // splits
							// a
							// string
							// by any amount of whitespace

							if (arr.length > 10)
								JSON.receiveJSON(arr);

							if ((arr.length >= 13) && strToBool(arr[13]))
								enable = true;
							else
								enable = false;

							if (strToBool(arr[0]))
								init = true;

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

			public void run() {

				int i = 0;

				try {
					DataOutputStream os = new DataOutputStream(client.getOutputStream());

					while (true) {

						if (init) {
							// initalize web interface
							os.write(JSON.JSONInit().getBytes());
							init = false;
						}

						if (enable) {

							synchronized (lock2) {
								message = JSON.sendJSON();
							}
							buf = message.getBytes();

							// System.out.println(i + " sent " + message + "@"
							// + buf.length);

							i++;

							try {
								os.write(buf);
							} catch (IOException e) {
								// e.printStackTrace();
								System.out.println("Appears Client Closed the Connection");
								enable = false;

								start();
							}

						}

						try {
							Thread.sleep(200);
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

	public static boolean strToBool(String s) {
		// don't hard code your parameter.
		return s.equalsIgnoreCase("true");
	}

}