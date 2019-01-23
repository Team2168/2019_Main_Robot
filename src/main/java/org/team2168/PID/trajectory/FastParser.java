package org.team2168.PID.trajectory;

/**
 * Some speed-enhanced string parsing methods.
 * 
 * @author Jared341
 */
public class FastParser {
	// Assumes the format %.Nf, where N is constant and leading zeros are present.
	public static double parseFormattedDouble(String string) {
		// Find the decimal point
		int decimal = string.indexOf('.');

		if (decimal == -1 || decimal + 1 >= string.length()) {
			// The string is not formatted correctly.
			return 0.0;
		}

		long whole_part = Long.parseLong(string.substring(0, decimal));
		long fractional_part = Long.parseLong(string.substring(decimal + 1, string.length()));

		double divisor = Math.pow(10, string.length() - decimal - 1);
		double sign = ((string.indexOf('-') == -1) ? 1.0 : -1.0);

		return (double) whole_part + (double) fractional_part * sign / divisor;
	}
}