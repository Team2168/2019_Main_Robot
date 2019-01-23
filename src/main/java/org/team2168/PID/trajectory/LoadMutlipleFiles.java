package org.team2168.PID.trajectory;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.Timer;

public class LoadMutlipleFiles {

	// Make sure these match up!
	public static final int WALL_LANE_ID = 2;
	public final static String[] kPathNames = { "InsideLanePathFar", "CenterLanePathFar", "WallLanePath",
			"InsideLanePathClose", "StraightAheadPath", };
	public final static String[] kPathDescriptions = { "Inside, Far", "Middle Lane", "Wall Lane", "Inside, Close",
			"Straight ahead", };
	static Hashtable paths_ = new Hashtable();

	public static void loadPaths() {
		Timer t = new Timer();
		t.start();
		LoadPathFile deserializer = new LoadPathFile();
		// for (int i = 0; i < kPathNames.length; ++i) {
		//
		// TextFileReader reader = new TextFileReader("file://" + kPathNames[i] +
		// ".txt");
		//
		// Path path = deserializer.deserialize(reader.readWholeFile());
		// paths_.put(kPathNames[i], path);
		// }
		System.out.println("Parsing paths took: " + t.get());
	}

	public static Path get(String name) {
		return (Path) paths_.get(name);
	}

	public static Path getByIndex(int index) {
		return (Path) paths_.get(kPathNames[index]);
	}

}
