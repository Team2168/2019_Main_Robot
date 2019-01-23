package org.team2168.PID.trajectory;

/**
 * Interface for methods that deserializes a Path or Trajectory.
 * 
 * @author Jared341
 */
public interface IPathDeserializer {

	public Path deserialize(String serialized);
}