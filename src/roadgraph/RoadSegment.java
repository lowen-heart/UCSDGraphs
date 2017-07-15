package roadgraph;

import geography.GeographicPoint;
import gmapsfx.javascript.object.GoogleMap;

/**
 * @author Loris Previtali
 * 
 *         A class which represents a road segment (edge) of a graph based on geographic locations
 *
 */
public class RoadSegment {


	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	
	private String roadName;
	private String roadType;
	private double length;
	
	private GoogleMap g;

	/**
	 * Create a new RoadSegment with given parameters
	 * 
	 * @param start
	 * 				an intersection that is the starting point of a road segment
	 * @param end
	 *	 			an intersection that is the ending point of a road segment
	 * @param roadName
	 * 				road name of the road segment
	 * @param roadType
	 * 				road type of the road segment
	 * @param length
	 * 				length of the road segment
	 */
	public RoadSegment(GeographicPoint start, GeographicPoint end, String roadName, String roadType, double length) {
		this.startPoint = start;
		this.endPoint = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	// getters
	public GeographicPoint getStartPoint() {
		return startPoint;
	}
	
	public GeographicPoint getEndPoint() {
		return endPoint;
	}

	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getLength() {
		return length;
	}
	
}
