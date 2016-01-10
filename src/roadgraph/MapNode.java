/**
 * @author Justin Tumale
 * 
 * A class which represents each node in the graph, each node
 * being an intersection on a map.
 */

package roadgraph;
import java.util.*;

import geography.GeographicPoint;



/**
 * @author Justin Tumale
 * 
 * A class which represents a node of a geographic location
 * A node is an intersection in a graph
 *
 */
public class MapNode implements Comparable<MapNode>{
	
	double longitude, latitude;
	private List <MapEdge> edges;
	private List <MapNode> neighbors = new ArrayList<MapNode>();
	private double weight;
	
	
	/** 
	 * Create a new empty MapNode
	 */
	public MapNode(){
		longitude = Double.POSITIVE_INFINITY;
		latitude = Double.POSITIVE_INFINITY;
		edges = null;
		neighbors = null;
		this.setWeight(0);
	}
	
	/** 
	 * Create a new MapNode from a geographic point
	 * @param  p the geographic point
	 */	
	public MapNode(GeographicPoint p){
		longitude = p.x;
		latitude = p.y;
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Adds a directed edge to the graph from one point to another.
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
	public void implementAddEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(newEdge);
		
		MapNode node = new MapNode(to);
		neighbors.add(node);
	}
	

	/** Get the edges from a node.
	 * 
	 * @return The list of edges that intersect, start, or end from a node
	 */
	public List<MapEdge> getEdges(){
		List<MapEdge> tempList = new ArrayList<MapEdge>();
		tempList = edges;
		return tempList;
	}
	
	/** Get the coordinates from a node.
	 * 
	 * @return The coordinates of a node
	 */
	public GeographicPoint getCoordinates(){
		GeographicPoint point = new GeographicPoint(longitude, latitude);
		return point;
	}
	
	/** Get a node's neighboring nodes connected by an edge.
	 * 
	 * @return The list of a node's neighboring nodes.
	 */
	public List<MapNode> getNeighbors(){
		return this.neighbors;
	}

	/** Get a node's weight.
	 * 
	 * @return The weight of a node
	 */
	public double getWeight(){
		return this.weight;
	}
	
	/** Set a node's weight.
	 * 
	 * @param The weight of a node
	 */
	public void setWeight(double n){
		this.weight = n;
	}
	
	/** Compare the weights of two node's.
	 * 
	 * @param The node to compare to
	 */
	@Override
	public int compareTo(MapNode o) {
        if (this.getWeight() == o.getWeight()){
            return 0;
        }
        else if (this.getWeight() > o.getWeight()){
            return 1;
        }
        else return -1;
	}


}
