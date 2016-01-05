package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	GeographicPoint start;
	GeographicPoint end;
	String streetName;
	double distance;
	String roadType_;
	
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		start = from;
		end = to;
		streetName = roadName;
		distance = length;
		roadType_ = roadType;
	}
	
	public double getDistance(){
		return this.distance;
	}
}
