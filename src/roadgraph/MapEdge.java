/**
 * @author Justin Tumale
 * 
 * A class which represents each edge in the graph, each edge
 * being a street between two geographic points.
 */

package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	GeographicPoint start;
	GeographicPoint end;
	String streetName;
	double distance;
	String roadType_;
	
	
	/**
	 * Creates a MapEdge based on the given parameters
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		start = from;
		end = to;
		streetName = roadName;
		distance = length;
		roadType_ = roadType;
	}
	
	/** Get an edge's distance.
	 * 
	 * @return The distance of an edge
	 */
	public double getDistance(){
		return this.distance;
	}
}
