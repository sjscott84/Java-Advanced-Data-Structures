package roadgraph;

import java.util.List;
import java.util.LinkedList;
import geography.GeographicPoint;
import roadgraph.Edge;

public class Vertex {
	private List<Edge> edges;
	private GeographicPoint location;
	private double distance = Integer.MAX_VALUE;
	private double estDistance = Integer.MAX_VALUE;
	private double time = Integer.MAX_VALUE;
	private double estTime = Integer.MAX_VALUE;
	
	//Vertex constructor, creates a vertex with a location and a list of edges
	public Vertex(GeographicPoint location) {
		this.location = location;
		this.edges = new LinkedList<Edge>();
	}
	
	//When an edge is added, this method is called, and adds the edge to the list of edges
	public void addEdge(Edge street){
		edges.add(street);
	}
	
	//This method returns the list of edges
	public List<Edge> getEdges(){
		return edges;
	}
	
	//This method returns the location of the Vertex
	public GeographicPoint getLocation(){
		return location;
	}
	
	public double getDistance(){
		return distance;
	}
	
	public void setDistance(double newDistance){
		distance = newDistance;
	}
	
	public double getEstDistance(){
		return estDistance;
	}
	
	public void setEstDistance(double newDistance){
		estDistance = newDistance;
	}
	
	public double getTime(){
		return time;
	}
	
	public void setTime(double newTime){
		time = newTime;
	}
	
	public double getEstTime(){
		return estTime;
	}
	
	public void setEstTime(double newTime){
		estTime = newTime;
	}

}
