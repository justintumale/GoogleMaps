/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	HashMap <GeographicPoint, MapNode> vertices; //HashMap that maps each coordinate to a node
	int numVertices, numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapNode>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return null;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		numVertices++;
		MapNode n = new MapNode(location);
		//nodes.add(n);
		vertices.put(location, n);
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		MapNode a = vertices.get(from);
		//MapNode a = new MapNode(from);
		a.implementAddEdge(from, to, roadName, roadType, length);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		

		Queue<MapNode> q = new LinkedList<MapNode>(); 							//Initialize queue
		HashSet<MapNode> visited = new HashSet<MapNode>();						//Initialize visited HashSet
		HashMap <MapNode, MapNode> parent = new HashMap <MapNode, MapNode>();	//Initialize parent HashMap
		
		MapNode startNode = vertices.get(start); 	//get start node
		q.add(startNode);						 	//enqueue start node
		visited.add(startNode);						//add start node to visited
		
		// Hook for visualization.  See writeup.
		nodeSearched.accept(startNode.getCoordinates());
		
				
		while (q.isEmpty() == false){				//while queue is not empty
			
			MapNode dqNode = q.remove();				//dequeue current node from the front of the queue
			nodeSearched.accept(dqNode.getCoordinates());
			
			if ( vertices.get(goal) == vertices.get(dqNode.getCoordinates())){		//if current node = goal, then return the parent map
		
				List<GeographicPoint> listCoordinates = new ArrayList<GeographicPoint>();
				listCoordinates = buildPath(dqNode, parent);					//Build a list using the path found from the start node to the goal node.
				return listCoordinates;
				
			}
		
			for (MapEdge e : dqNode.getEdges()){					//for each of the current node's neighbors not in the visited set	
				MapNode n = vertices.get(e.end);
				if(visited.contains(n) == false){
					visited.add(n);
					parent.put(n, dqNode);
					q.add(n);
				}
			}
		}//while
		return null;
	}
	
	
	/** Build a list using the path found from the start node to the goal node.
	 * 
	 * @param dqNode The goal node
	 * @param parent The hashmap of every node's parent
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> buildPath(MapNode dqNode, HashMap <MapNode, MapNode> parent){
		List<MapNode> mapPath = new ArrayList<MapNode>();
		List<GeographicPoint> listCoordinates = new ArrayList<GeographicPoint>();
						
		boolean loop = true;
		while (loop){
			mapPath.add(dqNode);
			dqNode = parent.get(dqNode);
			if (parent.get(dqNode) == null){
				mapPath.add(dqNode);
				loop = false;
			}
		}
		
		Collections.reverse(mapPath);
		
		for (MapNode n : mapPath){
			listCoordinates.add(n.getCoordinates());
		}
		return listCoordinates;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		PriorityQueue <MapNode> pq = new PriorityQueue<MapNode>(); //Initialize priority queue
		HashSet <MapNode> visited = new HashSet<MapNode>();
		HashMap <MapNode, MapNode> parent = new HashMap <MapNode, MapNode>();	//Initialize parent HashMap
		
		//Initialize values to infinity
		MapNode startNode = vertices.get(start);
		initializeNodes(startNode, vertices);
		
		//Enqueue start node
		pq.add(startNode);						 				
		
		//while queue is not empty
		while (pq.isEmpty() == false){	
			
			//dequeue current node from the front of the queue
			MapNode dqNode = pq.remove();						
			nodeSearched.accept(dqNode.getCoordinates());
			
			//if curr is not visited
			if (visited.contains(dqNode) == false){	
				
				//add curr to the visited set
				visited.add(dqNode);			
				
				//if the current node is equal to the goal node...
				if ( vertices.get(dqNode.getCoordinates()) == vertices.get(goal)){
					List<GeographicPoint> listCoordinates = new ArrayList<GeographicPoint>();
					
					//Build a list using the path found from the start node to the goal node.
					listCoordinates = buildPath(dqNode, parent);
					
					//Print out the visited nodes
					printPathOfCoordinates(listCoordinates);
					return listCoordinates;
				}
				
				//for each of the current node's neighbors not in the visited set	
				for (MapEdge e : dqNode.getEdges()){						
					MapNode n = vertices.get(e.end);
					
					if(visited.contains(n) == false){
						//visited.add(vertices.get(e.end));
						//1. if path through curr to n is shorter
						double newWeight = dqNode.getWeight() + e.getDistance();
						if (newWeight < n.getWeight())	{			
							//2. update n's distances
							n.setWeight(newWeight);
							//3. update curr as n's parent in parent map
							parent.put(n, dqNode);
							//4. enqueue (n, distance) into the PQ
							pq.add(n);
						}
					}
				}
				
			}//if
		}//while
		
		return null;
	}
	
	/**Initialize the vertices before running Dijkstra;s algorithm.  The start node's weight is initialized to 0,
	 * and every other vertex's weight is initialized to infinity.
	 * @param startNode The starting node
	 * @param vertices	Every vertex that is not the start node
	 */
	public void initializeNodes(MapNode startNode, HashMap <GeographicPoint, MapNode> vertices){
		for (MapNode n : vertices.values()){
			n.setWeight(Double.POSITIVE_INFINITY);
		}
		startNode.setWeight(0);
	}
	
	/** Print out the coordinates of the path that Dijkstra's algorithm takes.
	 * 
	 * @param listCoordinates The list of coordinates in sequence that make up
	 * the shortest path from start to goal
	 */
	public void printPathOfCoordinates(List<GeographicPoint> listCoordinates){
		for (GeographicPoint p : listCoordinates){
			System.out.println(p);
		}
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//Initialize priority queue
		PriorityQueue <MapNode> pq = new PriorityQueue<MapNode>(); //Initialize priority queue
		
		//Initialize HashSet
		HashSet <MapNode> visited = new HashSet<MapNode>();
		HashMap <MapNode, MapNode> parent = new HashMap <MapNode, MapNode>();	//Initialize parent HashMap
		
		//Initialize values to infinity
		MapNode startNode = vertices.get(start);
		initializeNodes(startNode, vertices);
		
		//Initialize estimated distance
		double heuristicDistance = start.distance(goal);
		
		//Enqueue start node
		pq.add(startNode);						 				
		
		//while queue is not empty
		while (pq.isEmpty() == false){	
			
			//dequeue current node from the front of the queue
			MapNode dqNode = pq.remove();						
			nodeSearched.accept(dqNode.getCoordinates());
			
			//if curr is not visited
			if (visited.contains(dqNode) == false){	
				
				//add curr to the visited set
				visited.add(dqNode);			
				
				//if the current node is equal to the goal node...
				if ( vertices.get(dqNode.getCoordinates()) == vertices.get(goal)){
					List<GeographicPoint> listCoordinates = new ArrayList<GeographicPoint>();
					
					//Build a list using the path found from the start node to the goal node.
					listCoordinates = buildPath(dqNode, parent);
					
					//Print out the visited nodes
					printPathOfCoordinates(listCoordinates);
					return listCoordinates;
				}
				
				//for each of the current node's neighbors not in the visited set	
				for (MapEdge e : dqNode.getEdges()){						
					MapNode n = vertices.get(e.end);
					
					if(visited.contains(n) == false){
						//set the current node as it's neighboring nodes' parent
						parent.put(n,  dqNode);
						
						//calculate the distance from the start node to the current node,
						//to each neighboring node
						double g = dqNode.getWeight() + e.getDistance();
						//calculate the estimated distance from the neighboring node to 
						//the goal
						double h = n.getCoordinates().distance(goal);
						//set the weight of the neighboring node
						n.setWeight(  g + h);
						
						//if the original heuristic distance is less than or equal
						//to the new heuristic distance, add the node to the 
						//priority queue.
						if ( heuristicDistance <= n.getWeight()){
							pq.add(n);
						}
							
					}
				}
				
			}//if
		}//while
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(1, 1);
		GeographicPoint goal = new GeographicPoint(8, -1);
		
		System.out.println("Dikstra's: ");
		theMap.dijkstra(start,  goal);
		
		System.out.println("A star: ");
		theMap.aStarSearch(start, goal);
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
