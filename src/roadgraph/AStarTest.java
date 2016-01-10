package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

public class AStarTest {
	
	public static void main(String[]args){
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");
		
		GeographicPoint path1_1 = new GeographicPoint(7, 3);
		GeographicPoint path1_2 = new GeographicPoint(8, -1);
		GeographicPoint path1_3 = new GeographicPoint(4, -1);
		
		GeographicPoint path2_1 = new GeographicPoint(7, 3);
		GeographicPoint path2_2 = new GeographicPoint(4, -1);
		GeographicPoint path2_3 = new GeographicPoint(4, 0);
		GeographicPoint path2_4 = new GeographicPoint(4, -1);
		
		double path1 = path1_1.distance(path1_2) + path1_2.distance(path1_3);
		
		double path2 = path2_1.distance(path2_2) + path2_2.distance(path2_3) + path2_3.distance(path2_4);
		
		System.out.println("path 1: " + path1);
		System.out.println("path 2: " + path2);
		
		

		
	}

}
