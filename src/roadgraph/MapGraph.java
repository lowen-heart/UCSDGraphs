/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Loris Previtali
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	// This HashMap associate all the intersections with their road segments.
	// It contains as a key an intersection of the graph of type GeographicPoint
	// and
	// as a value a list of RoadSegment, a list that represents all the edges
	// starting from that intersection.
	// RoadSegment is a class added to roadgraph starter package
	HashMap<GeographicPoint, List<RoadSegment>> intersections;
	HashMap<GeographicPoint, MapNode> vertices;
	int numVertices;
	int numEdges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		intersections = new HashMap<GeographicPoint, List<RoadSegment>>();
		vertices = new HashMap<GeographicPoint,MapNode>();
		numVertices = 0;
		numEdges = 0;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return vertices.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return vertices.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {

		if (location == null || vertices.containsKey(location)) {
			return false;
		}
		// add the new intersection inside intersection HashMap and create an
		// empty list of RoadSegment
		intersections.put(location, new LinkedList<RoadSegment>());
		vertices.put(location, new MapNode(location));
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// instead of doing a big long complicated if I decided to divide input
		// control in 3 separate if
		// divided by scope

		if (!intersections.containsKey(from) || !intersections.containsKey(to)) {
			throw new IllegalArgumentException();
		}

		if (from == null || to == null || roadName == null || roadType == null) {
			throw new IllegalArgumentException();
		}

		if (length < 0) {
			throw new IllegalArgumentException();
		}

		RoadSegment street = new RoadSegment(from, to, roadName, roadType, length);

		// get the road segments (edges) already present for that node
		List<RoadSegment> edges = intersections.get(from);
		// add the last road segment created to the list of road segments and
		// put it back to intersections
		edges.add(street);
		intersections.put(from, edges);
		numEdges++;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		// if start or goal is null just return a null value
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		// create parent hashmap
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();

		// call bfsSearch sub-routine to do bfs search on the graph
		boolean found = bfsSearch(start, goal, parent);

		// if the path does not exists, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}

		// reconstruct the path
		return buildPath(start, goal, parent);
	}

	/**
	 * Find neighbors given a geographic point
	 * 
	 * @param point
	 *            The geographic point of which search its neighbors
	 *
	 * @return The list of neighbors. It returns null if the point parameter is
	 *         not an intersection in the graph or is null.
	 */
	public List<GeographicPoint> getNeighbors(GeographicPoint point) {

		if (!vertices.containsKey(point) || point == null) {
			System.out.println("The geographic point is not part of the graph or is null. No neighbors found.");
			return null;
		}
		List<GeographicPoint> neighbors = new LinkedList<GeographicPoint>();
		// get the road segments (edges) of the given point
		List<RoadSegment> edges = intersections.get(point);
		// for each edge take the end point and add it to the neighbors list
		for (RoadSegment s : edges) {
			neighbors.add(s.getEndPoint());
		}
		return neighbors;
	}

	/**
	 * Find if exist a path between a two given geographic points
	 * 
	 * @param start
	 *            The starting geographic point where start
	 * @param end
	 *            The ending target geographic point where arrive
	 * @param parent
	 *            The parent HashMap
	 *
	 * @return True if there is a path, false if there is no path between the
	 *         two geographic points
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint end,
			HashMap<GeographicPoint, GeographicPoint> parent) {

		// create a queue where put geographic points to visit
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		// hashset of geographic points visited
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();

		queue.add(start);
		visited.add(start);
		GeographicPoint curr;

		// while there is no more geographic points to analyze
		while (!queue.isEmpty()) {

			curr = queue.poll();

			if (curr.equals(end)) {
				return true;
			}

			// get the neighbors of the current geographic point
			List<GeographicPoint> neighbors = getNeighbors(curr);
			ListIterator<GeographicPoint> neighIter = neighbors.listIterator(neighbors.size());

			while (neighIter.hasPrevious()) {
				GeographicPoint next = neighIter.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parent.put(curr, next);
					queue.add(next);
				}
			}
		}

		return false;
	}

	/**
	 * Reconstruct the path between two geographic points
	 * 
	 * @param start
	 *            The starting geographic point where start
	 * @param end
	 *            The ending target geographic point where arrive
	 * @param parent
	 *            The parent HashMap
	 *
	 * @return return a list of geographic points that represents the path
	 *         between two intersections
	 */
	private List<GeographicPoint> buildPath(GeographicPoint start, GeographicPoint end,
			HashMap<GeographicPoint, GeographicPoint> parent) {

		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = start;
		GeographicPoint prev = null;
		// while we have not reached the start add the current node visited as
		// first node inside the path
		// doing so we are creating the path from the start to the end
		while (curr != null) {
			//path.addFirst(curr);
			path.add(curr);
			//add parent list to visitedPaths inside MapNode.
			this.vertices.get(curr).addVisitedPath(end, new HashMap<GeographicPoint, GeographicPoint>(parent));
			prev = curr;
			curr = parent.get(curr);
			parent.remove(prev);
		}
		// add start to the begin
		//path.addFirst(start);
		path.add(end);
		System.out.println(path);
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		// if start or goal is null just return a null value
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		// create parent hashmap
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();

		// call bfsSearch sub-routine to do bfs search on the graph
		boolean found = dijkstraSearch(start, goal, parent,nodeSearched);

		// if the path does not exists, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}

		// reconstruct the path
		return buildPath(start, goal, parent);
	}

	/**
	 * Find if exist a path between a two given geographic points
	 * 
	 * @param start
	 *            The starting geographic point where start
	 * @param end
	 *            The ending target geographic point where arrive
	 * @param parent
	 *            The parent HashMap
	 *
	 * @return True if there is a path, false if there is no path between the
	 *         two geographic points
	 */
	private boolean dijkstraSearch(GeographicPoint start, GeographicPoint end,
			HashMap<GeographicPoint, GeographicPoint> parent,Consumer<GeographicPoint> nodeSearched) {

		// create a queue where put geographic points to visit
		PriorityQueue<MapNode> pqueue = new PriorityQueue<MapNode>();
		// hashset of geographic points visited
		HashSet<MapNode> visited = new HashSet<MapNode>();

		// set all vertices to infinite and add them to nodes
		Set<GeographicPoint> vertices = getVertices();
		for (GeographicPoint g : vertices) {
			this.vertices.get(g).setDistance(Double.POSITIVE_INFINITY);
			this.vertices.get(g).setPredDistance(0.0);
		}

		this.vertices.get(start).setDistance(0.0);
		pqueue.add(this.vertices.get(start));
		MapNode curr;
		int counter = 0;

		// while there is no more geographic points to analyze
		while (!pqueue.isEmpty()) {

			curr = pqueue.poll();
			nodeSearched.accept(curr.getLoc());
			//System.out.println("DIJKSTRA visiting" + curr);
			counter++;
			

			if (!visited.contains(curr)) {

				visited.add(curr);

				HashMap<GeographicPoint,GeographicPoint> path = this.vertices.get(curr.getLoc()).findPath(end);
				if(path != null){
					System.out.println("FASTER");
					parent.putAll(path);
					curr.setLoc(end);
				}
				
				if (curr.getLoc().equals(end)) {
					System.out.println("Dijkstra nodes visited in search: " + counter);
					return true;
				}

				// get the neighbors of the current geographic point
				List<GeographicPoint> neighbors = getNeighbors(curr.loc);
				ListIterator<GeographicPoint> neighIter = neighbors.listIterator(neighbors.size());

				while (neighIter.hasPrevious()) {
					// if path through curr to n is shorter
					GeographicPoint next = neighIter.previous();

					double distance = curr.distance;
					List<RoadSegment> edges = intersections.get(curr.loc);
					for (RoadSegment e : edges) {
						if (e.getEndPoint() == next) {
							distance += e.getLength();
							break;
						}
					}
				
					if (distance < this.vertices.get(next).getFullDistance()) {
						parent.put(curr.loc,next);
						this.vertices.get(next).setDistance(distance);
						pqueue.add(this.vertices.get(next));
					}
				}
			}

		}

		return false;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		// if start or goal is null just return a null value
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		// create parent hashmap
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();

		// call bfsSearch sub-routine to do bfs search on the graph
		boolean found = aStarSearch(start, goal, parent,nodeSearched);

		// if the path does not exists, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}

		// reconstruct the path
		return buildPath(start, goal, parent);
	}

	/**
	 * Find if exist a path between a two given geographic points
	 * 
	 * @param start
	 *            The starting geographic point where start
	 * @param end
	 *            The ending target geographic point where arrive
	 * @param parent
	 *            The parent HashMap
	 *
	 * @return True if there is a path, false if there is no path between the
	 *         two geographic points
	 */
	private boolean aStarSearch(GeographicPoint start, GeographicPoint end,
			HashMap<GeographicPoint, GeographicPoint> parent,Consumer<GeographicPoint> nodeSearched) {

		// create a queue where put geographic points to visit
		PriorityQueue<MapNode> pqueue = new PriorityQueue<MapNode>();
		// hashset of geographic points visited
		HashSet<MapNode> visited = new HashSet<MapNode>();

		// set all vertices to infinite and add them to nodes
		Set<GeographicPoint> vertices = getVertices();
		for (GeographicPoint g : vertices) {
			this.vertices.get(g).setDistance(Double.POSITIVE_INFINITY);
			this.vertices.get(g).setPredDistance(Double.POSITIVE_INFINITY);
		}

		this.vertices.get(start).setDistance(0.0);
		this.vertices.get(start).setPredDistance(0.0);
		pqueue.add(this.vertices.get(start));
		MapNode curr;
		int counter = 0;

		// while there is no more geographic points to analyze
		while (!pqueue.isEmpty()) {

			curr = pqueue.poll();
			nodeSearched.accept(curr.getLoc());
			//System.out.println("ASTAR visiting" + curr);
			counter++;

			if (!visited.contains(curr)) {

				visited.add(curr);
				
				HashMap<GeographicPoint,GeographicPoint> path = this.vertices.get(curr.getLoc()).findPath(end);
				if(path != null){
					System.out.println("FASTER");
					parent.putAll(path);
					curr.setLoc(end);
				}

				if (curr.getLoc().equals(end)) {
					System.out.println("AStar nodes visited in search: " + counter);
					return true;
				}

				// get the neighbors of the current geographic point
				List<GeographicPoint> neighbors = getNeighbors(curr.loc);
				ListIterator<GeographicPoint> neighIter = neighbors.listIterator(neighbors.size());

				while (neighIter.hasPrevious()) {
					// if path through curr to n is shorter
					GeographicPoint next = neighIter.previous();

					double distance = curr.distance;
					double preddistance = 0;
					List<RoadSegment> edges = intersections.get(curr.loc);
					for (RoadSegment e : edges) {
						if (e.getEndPoint() == next) {
							distance += e.getLength();
							preddistance = next.distance(end);
							break;
						}
					}
				
					if ((distance + preddistance) < this.vertices.get(next).getFullDistance()) {
						parent.put(curr.loc,next);
						this.vertices.get(next).setDistance(distance);
						this.vertices.get(next).setPredDistance(preddistance);
						pqueue.add(this.vertices.get(next));
					}
					
				}
			}

		}

		return false;
	}

	public void printGraph() {
		System.out.println("Nodes: " + getNumVertices() + " Edges: " + getNumEdges());
		System.out.println("Vertices: " + getVertices());
	}

	public static void main(String[] args) {
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.
		firstMap.printGraph();

		// Check num of vertices and edges
		System.out.println("Vertices:" + firstMap.getNumVertices() + " Edges: " + firstMap.getNumEdges());
		
		// Check if adding a point that is already present returns false
		GeographicPoint point = new GeographicPoint(1.0, 1.0);
		System.out.println("AddVertex must return false: " + firstMap.addVertex(point));

		GeographicPoint from = new GeographicPoint(5.0, 10.0);
		GeographicPoint to = new GeographicPoint(5.0, 6.0);

		try {
			firstMap.addEdge(from, point, "Piatto Ln", "Street", 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on first node not in hashmap");
		}
		try {
			firstMap.addEdge(point, to, "Piatto Ln", "Street", 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on second node not in hashmap" + e.getMessage());
		}
		try {
			firstMap.addEdge(null, point, "Piatto Ln", "Street", 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on first node null");
		}
		try {
			firstMap.addEdge(point, null, "Piatto Ln", "Street", 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on second node null");
		}
		try {
			firstMap.addEdge(point, point, null, "Street", 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on third param null");
		}
		try {
			firstMap.addEdge(point, point, "Piatto ln", null, 10);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on forth param null");
		}
		try {
			firstMap.addEdge(point, point, "Piatto ln", "Street", -9);
		} catch (IllegalArgumentException e) {
			System.out.println("IllegalArgumentExpression on length negative");
		}*/

		/*
		 * Here are some test cases you should try before you attempt the Week 3
		 * End of Week Quiz, EVEN IF you score 100% on the programming
		 * assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		testStart.setLocation(4.0, 1.0);
		
		testroute = simpleTestMap.dijkstra(testStart, testEnd);
		testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		/* Use this code in Week 3 End of Week Quiz */

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		start.setLocation(32.8649272, -117.229097);
		
		route = theMap.dijkstra(start, end);
		route2 = theMap.aStarSearch(start,end);

	}

}
