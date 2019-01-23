package org.team2168.utils;

public interface TCPMessageInterface {
	/**
	 * 
	 * @return a string in JSON format (including brackets and quotation marks).
	 */
	public String sendJSON();

	/**
	 * 
	 * @param message
	 *            is a string array, where the positions of each element correspond
	 *            to the values of the JSON key/value pair.
	 */
	public void receiveJSON(String[] message);

	public String JSONInit();
}