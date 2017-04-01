/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Comparator;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, Vertex> intersections;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		intersections = new HashMap<GeographicPoint, Vertex>();
		numEdges = 0;
	}
	
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		int size = intersections.size();
		return size;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> intersectionLocations = new  HashSet<GeographicPoint>();
		for(Vertex v: intersections.values()){
			intersectionLocations.add(v.getLocation());
		}
		return intersectionLocations;
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
		
		if(location == null || intersections.get(location) != null){
			return false;
		}else{	
			Vertex current = new Vertex(location);
			intersections.put(location, current);
			return true;
		}
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
		Vertex testStart = intersections.get(from);
		Vertex testEnd = intersections.get(to);
		if(from == null || to == null || roadName == null || roadType == null || length < 0 || testStart == null || testEnd == null){
			throw new IllegalArgumentException();
		}else{
			Edge current = new Edge(from, to, roadName, roadType, length);
			testStart.addEdge(current);
			numEdges++;
		}
		
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
		//Initialise a queue, a hashset, a hashmap and a list for this method
		Queue<Vertex> stack = new LinkedList<Vertex>();
		HashSet<Vertex> visited = new HashSet<Vertex>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		
		//Retrieve the start Vertex based on the start location
		Vertex startVertex = intersections.get(start);
		//Add the startVertex to the queue and and hashset
		stack.add(startVertex);
		visited.add(startVertex);
		//Start searching the graph
		while(!stack.isEmpty()){
			Vertex current = stack.remove();
			//If current location matches the goal location then build the path and return the completed path
			if(current.getLocation().equals(goal)){
				GeographicPoint currentLocation = current.getLocation();
				path.add(currentLocation);
				while(!currentLocation.equals(start)){
					GeographicPoint parent = parents.get(currentLocation);
					path.add(0, parent);
					currentLocation = parent;
				}
				return path;
			}
			//Add each of current vertex's edges to the queue and the visited hashset
			for(Edge n: current.getEdges()){
				GeographicPoint next = n.getEnd();
				Vertex v = intersections.get(next);
				if(!visited.contains(v)){
					visited.add(v);
					//This adds the current and next locations (or parent and child locations) to a hashmap that can be traversed to retrieve the path
					parents.put(next, current.getLocation());
					stack.add(v);
				}
			}
		}	
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		return null;
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
		PriorityQueue<Vertex> pq = new PriorityQueue<Vertex>(10, new VertexComparator());
		HashSet<Vertex> visited = new HashSet<Vertex>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		int nodeSearchedCount = 0;
		
		for(Vertex v: intersections.values()){
			v.setDistance(Integer.MAX_VALUE);
		}
		
		Vertex startVertex = intersections.get(start);
		pq.add(startVertex);
		startVertex.setDistance(0);
		while(!pq.isEmpty()){
			Vertex curr = pq.remove();
			nodeSearched.accept(curr.getLocation());
			nodeSearchedCount++;
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal)){
					System.out.println("diklstra count "+nodeSearchedCount);
					GeographicPoint currentLocation = curr.getLocation();
					path.add(currentLocation);
					while(!currentLocation.equals(start)){
						GeographicPoint parent = parents.get(currentLocation);
						path.add(0, parent);
						currentLocation = parent;
					}
					return path;
				}
				for(Edge e: curr.getEdges()){
					GeographicPoint next = e.getEnd();
					Vertex v = intersections.get(next);
					if(!visited.contains(v)){
						double newDistance = curr.getDistance() + e.getLength();
						if(newDistance < v.getDistance()){
							v.setDistance(newDistance);
							parents.put(next, curr.getLocation());
							pq.add(v);	
						}
					}
				}
				
			}
		}
		

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
	
	public List<GeographicPoint> aStarSearchTimeTaken(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearchTimeTaken(start, goal, temp);
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
		PriorityQueue<Vertex> pq = new PriorityQueue<Vertex>(10, new aStarVertexComparator());
		HashSet<Vertex> visited = new HashSet<Vertex>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		int nodeSearchedCount = 0;
		
		for(Vertex v: intersections.values()){
			v.setDistance(Integer.MAX_VALUE);
			v.setEstDistance(Integer.MAX_VALUE);
		}
		
		Vertex startVertex = intersections.get(start);
		pq.add(startVertex);
		startVertex.setDistance(0);
		startVertex.setEstDistance(0);
		while(!pq.isEmpty()){
			Vertex curr = pq.remove();
			nodeSearchedCount ++;
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal)){
					System.out.println("astar count "+nodeSearchedCount);
					GeographicPoint currentLocation = curr.getLocation();
					path.add(currentLocation);
					while(!currentLocation.equals(start)){
						GeographicPoint parent = parents.get(currentLocation);
						path.add(0, parent);
						currentLocation = parent;
					}
					return path;
				}
				for(Edge e: curr.getEdges()){
					GeographicPoint next = e.getEnd();
					Vertex neighbor = intersections.get(next);
					if(!visited.contains(neighbor)){
						double estDistance = next.distance(goal);
						double newDistance = curr.getDistance() + e.getLength();
						if((newDistance + estDistance) <  neighbor.getEstDistance()){
							neighbor.setDistance(newDistance);
							neighbor.setEstDistance(newDistance + estDistance);
							parents.put(next, curr.getLocation());
							pq.add(neighbor);	
						}
					}
				}
				
			}
		}
		
		return null;
	}
	
	public List<GeographicPoint> aStarSearchTimeTaken(GeographicPoint start, 
			 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		PriorityQueue<Vertex> pq = new PriorityQueue<Vertex>(10, new aStarVertexComparator());
		HashSet<Vertex> visited = new HashSet<Vertex>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<GeographicPoint, GeographicPoint>();
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		int nodeSearchedCount = 0;
		
		for(Vertex v: intersections.values()){
			v.setDistance(Integer.MAX_VALUE);
			v.setEstDistance(Integer.MAX_VALUE);
		}
		
		Vertex startVertex = intersections.get(start);
		pq.add(startVertex);
		startVertex.setTime(0);
		startVertex.setEstTime(0);
		while(!pq.isEmpty()){
			Vertex curr = pq.remove();
			nodeSearchedCount ++;
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal)){
					System.out.println("astar count "+nodeSearchedCount);
					GeographicPoint currentLocation = curr.getLocation();
					path.add(currentLocation);
					while(!currentLocation.equals(start)){
						GeographicPoint parent = parents.get(currentLocation);
						path.add(0, parent);
						currentLocation = parent;
					}
					return path;
				}
				for(Edge e: curr.getEdges()){
					GeographicPoint next = e.getEnd();
					Vertex neighbor = intersections.get(next);
					double speed = e.getSpeed();
					if(!visited.contains(neighbor)){
						double estTime = next.distance(goal)/speed;
						double newTime = curr.getTime() + (e.getLength()/speed);
						if((newTime + estTime) <  neighbor.getEstTime()){
							neighbor.setTime(newTime);
							neighbor.setEstTime(newTime + estTime);
							parents.put(next, curr.getLocation());
							pq.add(neighbor);	
						}
					}
				}
				
			}
		}
			
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		
		System.out.println(firstMap.aStarSearchTimeTaken(start, end));
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);*/

		
		
	}
	
}
