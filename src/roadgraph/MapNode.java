package roadgraph;

import java.util.HashMap;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode> {

	GeographicPoint loc;
	HashMap<GeographicPoint,HashMap<GeographicPoint,GeographicPoint>> visitedPaths;
	double distance;
	double preddistance;

	public MapNode(GeographicPoint loc) {
		this(loc, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	}

	public MapNode(GeographicPoint loc, double distance) {
		this(loc, distance, Double.POSITIVE_INFINITY);
	}

	public MapNode(GeographicPoint loc, double distance, double preddistance) {
		this.loc = loc;
		this.distance = distance;
		this.preddistance = preddistance;
		this.visitedPaths = new HashMap<GeographicPoint,HashMap<GeographicPoint,GeographicPoint>>();
	}

	public boolean equals(MapNode obj) {
		return loc.equals(obj.loc);
	}

	public GeographicPoint getLoc() {
		return loc;
	}

	public void setLoc(GeographicPoint loc) {
		this.loc = loc;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	public double getPredDistance() {
		return preddistance;
	}

	public void setPredDistance(double preddistance) {
		this.preddistance = preddistance;
	}
	
	public double getFullDistance(){
		return (distance + preddistance);
	}
	
	public boolean addVisitedPath(GeographicPoint end, HashMap<GeographicPoint,GeographicPoint> visitedPath){
		this.visitedPaths.put(end, visitedPath);
		return true;
	}
	
	public HashMap<GeographicPoint,GeographicPoint> findPath(GeographicPoint node){
		return visitedPaths.get(node);
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + loc + ")]";
		toReturn += "Actual: " + distance + " Predicted: " + getFullDistance();
		return toReturn;
	}
	
	@Override
	public int compareTo(MapNode o) {
		// TODO Auto-generated method stub
		double distf1 = this.distance + this.preddistance;
		double distf2 = o.getDistance() + o.getPredDistance();
		if (distf1 > distf2) {
			return 1;
		} else if (distf1 < distf2) {
			return -1;
		}
		return 0;
	}

}
