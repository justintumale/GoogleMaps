package roadgraph;
import java.util.*;

import geography.GeographicPoint;

public class MapNodeAStar implements Comparable<MapNodeAStar>{
	
	double longitude, latitude;
	private List <MapEdge> edges;
	private List <MapNodeAStar> neighbors = new ArrayList<MapNodeAStar>();
	private double weight;
	private double function;
	
	public MapNodeAStar(){
	
	}
	
	public MapNodeAStar(GeographicPoint p){
		longitude = p.x;
		latitude = p.y;
		edges = new ArrayList<MapEdge>();
	}
	
	public void implementAddEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(newEdge);
		
		MapNodeAStar node = new MapNodeAStar(to);
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
	
	public List<MapNodeAStar> getNeighbors(){
		return this.neighbors;
	}

	public double getWeight(){
		return this.weight;
	}
	public void setWeight(double n){
		this.weight = n;
	}
	
	@Override
	public int compareTo(MapNodeAStar o) {
        if (getWeight() == o.getWeight()){
            return 0;
        }
        else if (getWeight() > o.weight){
            return 1;
        }
        else return -1;
	}

}
