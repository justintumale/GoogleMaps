package roadgraph;
import java.util.*;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	
	double longitude, latitude;
	private List <MapEdge> edges;
	private List <MapNode> neighbors = new ArrayList<MapNode>();
	private double weight;
	
	
	public MapNode(){
		weight = 0;
	}
	
	public MapNode(GeographicPoint p){
		longitude = p.x;
		latitude = p.y;
		edges = new ArrayList<MapEdge>();
	}
	
	public void implementAddEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(newEdge);
		
		MapNode node = new MapNode(to);
		neighbors.add(node);
	}
	
	public List<MapEdge> getEdges(){
		List<MapEdge> tempList = new ArrayList<MapEdge>();
		tempList = edges;
		return tempList;
	}
	
	public GeographicPoint getCoordinates(){
		GeographicPoint point = new GeographicPoint(longitude, latitude);
		return point;
	}
	
	public List<MapNode> getNeighbors(){
		return this.neighbors;
	}

	public double getWeight(){
		return this.weight;
	}
	public void setWeight(double n){
		this.weight = n;
	}
	
	@Override
	public int compareTo(MapNode o) {
        if (getWeight() == o.getWeight()){
            return 0;
        }
        else if (getWeight() > o.weight){
            return 1;
        }
        else return -1;
	}

}
