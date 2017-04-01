package roadgraph;

import geography.GeographicPoint;

public class Edge {
	
	private GeographicPoint start;
	private GeographicPoint end;
	private String name;
	private String type;
	private double streetLength;
	private double speed;
	
	//Constructor to create the edge with the start, end, name, type and length
	public Edge(GeographicPoint start, GeographicPoint end, String name, String type, double length) {
		this.start = start;
		this.end = end;
		this.name = name;
		this.type = type;
		this.streetLength = length;
		this.speed = getSpeedFromType(type);
	}
	
	private double getSpeedFromType(String type){
		double speed = 0;
		switch(type){
			case "motorway": speed = 500;
				break;
			case "trunk": speed = 55;
				break;
			case "primary": speed = 55;
				break;
			case "secondary": speed = 55;
				break;
			case "tertiary": speed = 45;
				break;
			case "unclassified": speed = 45;
				break;
			case "residential": speed = 30;
				break;
			case "living": speed = 25;
				break;
			case "service": speed = 15;
				break;
			default: speed = 35;
				break;
		}
		return speed;
	}
	
	//This method returns the start location of the edge
	public GeographicPoint getStart(){
		return start;
	}
	
	//This method returns the end location of the edge
	public GeographicPoint getEnd(){
		return end;
	}
	
	/*As this project only required the locations of the edges I have not writted methods to return the name, type or length
	 * however, those methods would go here if needed*/
	
	public double getLength(){
		return streetLength;
	}
	
	public String getType(){
		return type;
	}
	
	public double getSpeed(){
		return speed;
	}
}
