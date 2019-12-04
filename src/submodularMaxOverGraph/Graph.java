package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.PriorityQueue;
import java.util.Stack;
public class Graph<V> {
	private HashMap<V, ArrayList<Edge<V>>> adjacencyList;
	//private ArrayList<V> vertexList;
	private LinkedHashMap<V, Vertex> vertexList; //USE LinkedHashMap to preserve the odd. [hz 2.13.2016 fix random seed issue]
	private boolean directed;

	public Graph(boolean isDirected) {
		directed = isDirected;
		adjacencyList = new HashMap<V, ArrayList<Edge<V>>>();
		//vertexList = new ArrayList<V>();
		vertexList = new LinkedHashMap<V, Vertex>();
	}
	public void add(V vertex, ArrayList<Edge<V>> connectedVertices) {
		// Add the new vertex to the adjacencyList with it's list of connected nodes
		adjacencyList.put(vertex, connectedVertices);
		//vertexList.add(vertex);
		vertexList.put(vertex, new Vertex (vertex,0));		

		// If this is an undirected graph, every edge needs to represented
		// twice, once in the added vertex's list and once in the list of each
		// of the vertex's connected to the added vertex
		for (Edge<V> vertexConnectedToAddedVertex : connectedVertices) {
			ArrayList<Edge<V>> correspondingConnectedList = adjacencyList
					.get(vertexConnectedToAddedVertex.getVertex());
			// The added vertex's connections might not be represented in
			// the Graph yet, so we implicitly add them
			if (correspondingConnectedList == null) {
				adjacencyList.put(vertexConnectedToAddedVertex.getVertex(),
						new ArrayList<Edge<V>>());
				//vertexList.add(vertexConnectedToAddedVertex.getVertex());
				vertexList.put(vertexConnectedToAddedVertex.getVertex(), new Vertex (vertexConnectedToAddedVertex.getVertex(),0));

				correspondingConnectedList = adjacencyList
						.get(vertexConnectedToAddedVertex.getVertex());
			}
			if (!directed) {
				// The weight from one vertex back to another in an undirected
				// graph is equal
				double weight = vertexConnectedToAddedVertex.getWeight();
				correspondingConnectedList.add(new Edge<V>(vertex, weight));
			}
		}
	}

	//Directed Graph
	public boolean addArc(V source, V end, double weight) {
		if (!directed) {
			return false;
		}
		if (!adjacencyList.containsKey(source)) {
			ArrayList<Edge<V>> tempList = new ArrayList<Edge<V>>();
			tempList.add(new Edge<V>(end, weight));
			add(source, tempList);
			return true;
		}
		if (!adjacencyList.containsKey(end)) {
			ArrayList<Edge<V>> tempList = new ArrayList<Edge<V>>();
			add(end, tempList);
		}
		adjacencyList.get(source).add(new Edge<V>(end, weight));
		return true;
	}

	//Undirected Graph
	public boolean addEdge(V vertexOne, V vertexTwo, double weight) {
		if (directed) {
			return false;
		}
		if (!adjacencyList.containsKey(vertexOne)) {
			ArrayList<Edge<V>> tempList = new ArrayList<Edge<V>>();
			tempList.add(new Edge<V>(vertexTwo, weight));
			add(vertexOne, tempList);
			return true;
		}
		if (!adjacencyList.containsKey(vertexTwo)) {
			ArrayList<Edge<V>> tempList = new ArrayList<Edge<V>>();
			tempList.add(new Edge<V>(vertexOne, weight));
			add(vertexTwo, tempList);
			return true;
		}
		adjacencyList.get(vertexOne).add(new Edge<V>(vertexTwo, weight));
		adjacencyList.get(vertexTwo).add(new Edge<V>(vertexOne, weight));
		return true;
	}
	/**
	 * This method returns a list of all adjacent vertices of the give vertex without weight
	 *
	 * @param vertex the source vertex
	 * @return an array list containing the vertices
	 */
	public ArrayList<V> getAdjacentVertices(V vertex){
		ArrayList<V> returnList = new ArrayList<V>();
		for (Edge<V> edge : adjacencyList.get(vertex)) {
			returnList.add(edge.getVertex());
		}
		return returnList;
	}
	public double getDistanceBetween(V source, V end){
		//Special case: source and target are the same
		if(source.equals(end)) return 0;

		for (Edge<V> edge : adjacencyList.get(source)) {
			//if (edge.getVertex() == end){ //WARN: This cause an error. "==" NOT apply to strings.
			if (edge.getVertex().equals(end)){
				return edge.getWeight();
			}
		}
		return Double.POSITIVE_INFINITY;
	}	
	public LinkedHashMap<V, Vertex> getVertexList() {
		return vertexList;
	}
	public String toString() {
		String s = "";
		//for (V vertex : vertexList) {

		s+="Adjacency List:\n";
		for (V vertex : vertexList.keySet()) {	
			s += vertex.toString();
			s += " : ";
			s += adjacencyList.get(vertex);
			s += "\n";
		}

		s+="Verticies List:\n";
		for (Vertex vertex : vertexList.values()) {			
			s += vertex.toString();
			//s += "\n";
		}

		return s;
	}
	/**
	 * This method compute approximate cover cost using Christofides' algorithm
	 * @param start_node start vertex
	 * @param route_network underlying routing network used to obtain weight among vertices
	 * @param cover_nodes a set of vertices to be covered
	 * @return a tour start and end with start node along with cost
	 */
	public static <V> WrapWalkWithLength shortestCoverCostChristofides(V start_node, Graph<V> route_network, HashSet<V> cover_nodes){
		//System.out.println("Calling Cover Cost!!!");
		ArrayList <V> walk=new ArrayList<V> ();
		double length=0;

		//Build a complete subgraph G from route network which includes start vertex and covering nodes
		Graph<V> G =buildCompleteGraph(start_node, route_network, cover_nodes);				
		//System.out.println("Complete graph G:"+G.toString());

		//Special case: start node is the same as the node to be covered
		if(G.vertexList.size()==0) {
			walk.add(start_node);
			return new WrapWalkWithLength(walk, length);
		}

		//Calculate MST T of G
		Graph<V> T =growMST(start_node, G);		
		//System.out.println("MST T:"+T.toString());
		//Special case: MST not exist or some node is not reachable
		if(T.vertexList.size()==0||T.vertexList.size()<G.vertexList.size()){
			return new WrapWalkWithLength(walk, Double.POSITIVE_INFINITY);
		}

		//Calculate the set of vertices O with odd degree in T
		HashSet<V> oddDegreeVertices=getOddDegreeVertices(T);
		//System.out.println("Vertices with odd degree in T (O):"+oddDegreeVertices.toString());

		//Form the subgraph of G using only the vertices of O
		Graph<V> subG=subgraph(G, oddDegreeVertices);
		//System.out.println("Subgraph of G with O:"+subG.toString());

		//Construct a minimum-weight perfect matching M in this subgraph
		//Note: use simple greedy to this end, there is an exact algorithm available though.
		Graph<V> greedyM=greedyMatch(subG);
		//System.out.println("Perfect matching M:"+greedyM.toString());

		//Unite matching and spanning tree T\cup M to form an Eulerian multigraph
		Graph<V> uniteTM=uniteGraphs(T, greedyM);
		//System.out.println("T unite M:"+uniteTM.toString());

		//Calculate Euler tour
		ArrayList <V> eulerTour=searchEulerTour(start_node, uniteTM);
		//System.out.println("Euler tour:"+eulerTour);		

		//Remove repeated vertices, giving the algorithm's output
		ArrayList <V> tspPath=shortCutting(start_node, eulerTour);
		//System.out.println("TSP tour:"+tspPath);

		walk=tspPath;
		length=getWalkLength(G, tspPath); //USE G the complete graph to calculate length.

		return new WrapWalkWithLength(walk, length);
	}



	public static <V> double getWalkLength(Graph <V> graph, ArrayList<V> walk){
		double length=0;
		if(walk.size()>2) {
			for(int i=1; i<walk.size(); i++){
				length+=graph.getDistanceBetween(walk.get(i), walk.get(i-1));
				//System.out.println(walk.get(i)+":"+length);
			}
		}		

		return length;	
	}

	public static <V> ArrayList<V> shortCutting(V startNode, ArrayList<V> tour){
		ArrayList <V> cutRoute=new ArrayList <V> ();
		HashSet <V> visited=new HashSet<V>();

		for(V v: tour){
			if(!visited.contains(v)){
				visited.add(v);
				cutRoute.add(v);
			}
		}	

		cutRoute.add(startNode);
		return cutRoute;
	}

	public static <V> ArrayList<V> searchEulerTour(V origin, Graph <V> graph){
		ArrayList <V> eulerTour=new ArrayList<V> ();		
		Stack <V> liveVertices = new Stack<V>(); //Vertices that have uncovered edges
		HashSet <V> deadVertices = new HashSet<V>(); //Vertices that have NO uncovered edges
		eulerTour.add(origin);

		liveVertices.push(origin);

		while (!liveVertices.isEmpty()) { // Each iteration finds a circular route to expand current route
			V cv = liveVertices.pop();//start from a vertex from current route
			//System.out.println(cv);

			//Form a cycle starting and ending with cv 
			ArrayList <V> newCycle=new ArrayList<V> ();
			V pv=cv; //pointer			

			while(hasUncoveredEdge(graph, pv)){
				boolean backToStart=false;
				//Check if there is uncovered edge back to cv 
				if(!pv.equals(cv)){
					backToStart=isEdgeFeasible(graph, pv, cv);					
				}

				if(backToStart) break;

				//Check neighbors
				for(V nv:graph.getAdjacentVertices(pv)){
					if(!deadVertices.contains(nv)&&isEdgeFeasible(graph,pv,nv)){
						newCycle.add(nv);						
						//mark edge: pv-nv as covered
						marketEdgeAsCovered(graph, pv, nv);
						liveVertices.push(nv);

						//System.out.println(graph);
						pv=nv;
						break;
					}					
				}				
			}

			if(!pv.equals(cv)) {
				newCycle.add(cv);
				marketEdgeAsCovered(graph, pv, cv);	
			}

			//System.out.println("New cycle:"+cv+"-"+newCycle);

			if(newCycle.size()>0)
				eulerTour.addAll(eulerTour.indexOf(cv)+1, newCycle);


			if(!hasUncoveredEdge(graph, cv)) deadVertices.add(cv);
			else liveVertices.push(cv);
		}	

		return eulerTour;	
	}

	public static <V> boolean hasUncoveredEdge(Graph<V> graph, V v){
		boolean hasUcEdge=false;
		for(Edge e:graph.adjacencyList.get(v)){
			if(!e.isCovered()) {
				hasUcEdge=true;
				break;
			}
		}
		return hasUcEdge;
	}

	public static <V> boolean isEdgeFeasible(Graph <V> graph, V frm, V to){
		boolean isEdgeFeasible=false;
		for(Edge pve:graph.adjacencyList.get(frm)){
			//System.out.println("pv nb:"+pve.toString());
			if(pve.getVertex().equals(to)&&!pve.isCovered()){
				isEdgeFeasible=true;
				break;
			}
		}
		return isEdgeFeasible;
	}

	public static <V> void marketEdgeAsCovered(Graph <V> graph, V frm, V to){
		//pv->nv
		for(Edge pve:graph.adjacencyList.get(frm)){
			if(pve.getVertex().equals(to)&&!pve.isCovered()){
				pve.setCovered();
				break;
			}
		}
		//nv->pv
		for(Edge nve:graph.adjacencyList.get(to)){
			if(nve.getVertex().equals(frm)&&!nve.isCovered()){
				nve.setCovered();
				break;
			}
		}		

	}

	/**
	 * Unite two graphs
	 * @param g1 graph one
	 * @param g2 graph two
	 * @return a multigraph
	 */
	public static <V> Graph <V> uniteGraphs (Graph <V> g1, Graph <V> g2){
		Graph<V> U=new Graph<V> (true);		
		//copy first graph
		for(V v1: g1.getVertexList().keySet()){
			for(V w1: g1.getAdjacentVertices(v1)){
				U.addArc(v1, w1, g1.getDistanceBetween(v1, w1));
			}
		}
		//graph second graph
		for(V v2: g2.getVertexList().keySet()){
			for(V w2: g2.getAdjacentVertices(v2)){
				//if(U.getVertexList().containsKey(v2)&&U.getAdjacentVertices(v2).contains(w2))
				//continue;
				U.addArc(v2, w2, g2.getDistanceBetween(v2, w2));
			}
		}

		return U;
	}

	/**
	 * Find a perfect matching using shortest path
	 * @param graph
	 * @return Graph object representing perfect match
	 */
	public static <V> Graph <V> greedyMatch (Graph <V> graph){
		Graph<V> M=new Graph<V> (false);		

		//Sorting edge in increasing order
		//reference: https://github.com/faisal22/Christofides/blob/master/src/Christofides.java
		PriorityQueue<Edge<V>> Q = new PriorityQueue<>();

		for(V v: graph.getVertexList().keySet()){//2|E|
			for(Edge <V> e:graph.adjacencyList.get(v)){
				V eTo= e.getVertex();
				e.setFromVertex(v);
				Q.offer(e);//log|E|
			}
		}		

		while(!Q.isEmpty()){//|E|
			Edge <V> e=Q.poll();//log|E|
			//System.out.println(e.toString());
			V frm=e.getFromVertex();
			V to=e.getVertex();
			if(M.getVertexList().containsKey(frm)||M.getVertexList().containsKey(to))
				continue;
			M.addEdge(frm, to, graph.getDistanceBetween(frm, to));
		}		

		return M;
	}

	/**
	 * Get subgraph of a graph with given set of vertices
	 * @param graph graph
	 * @param includingNodes including vertices
	 * @return a subgraph
	 */
	public static <V> Graph<V> subgraph (Graph <V> graph, HashSet <V> includingNodes){
		Graph<V> subG=new Graph<V> (true);			
		for(V v: includingNodes){
			//System.out.println(v);
			for(V w: includingNodes){
				//System.out.println(w);
				if(w.equals(v)) continue;
				double dist=graph.getDistanceBetween(v, w);			
				subG.addArc(v, w, dist);
			}
		}	

		return subG;
	}


	/**
	 * Obtain a set of vertices with odd degree for given graph
	 * @param graph graph
	 * @return set of vertices with odd degree
	 */
	public static <V> HashSet getOddDegreeVertices(Graph<V> graph){
		HashSet<V> oddDegreeVertices=new HashSet<V> ();
		for(V v: graph.getVertexList().keySet()){
			if(graph.getAdjacentVertices(v).size()%2==1){
				oddDegreeVertices.add(v);
			}
		}		
		return oddDegreeVertices;	
	}

	/**
	 * Construct a minimum spanning tree given start node from a graph with Prim's algorithm using priority queue
	 * @param start_node start node
	 * @param graph the graph
	 * @return Graph object
	 */
	public static <V> Graph growMST(V start_node, Graph <V> graph){
		Graph <V> T = new Graph<V> (false);
		PriorityQueue<Vertex<V>> Q = new PriorityQueue<>();

		//Initialization
		for(V v: graph.getVertexList().keySet() ){//|V|
			if(v.equals(start_node))
				graph.getVertexList().get(v).min_dist=0;//start node
			else
				graph.getVertexList().get(v).min_dist=Double.POSITIVE_INFINITY;
			graph.getVertexList().get(v).pre_node=null;	
			Q.offer(graph.getVertexList().get(v));//log|V|
		}		

		//Main procedure
		while(!Q.isEmpty()){//|V|
			Vertex <V> vtx=Q.poll(); //|log|V||
			V v=vtx.getID();
			//System.out.println("Q.poll:"+v);

			if(vtx.pre_node!=null){
				V u=vtx.pre_node;
				T.addEdge(u, v, graph.getDistanceBetween(u, v));
			}
			
			for (V w : graph.getAdjacentVertices(vtx.getID())){//|2E| 
				Vertex wVtx=graph.getVertexList().get(w);
				if(Q.contains(wVtx)){
					double distVandW=graph.getDistanceBetween(v, w);
					//System.out.println(v+"-"+w+":"+distVandW);
					if(distVandW<wVtx.min_dist){
						wVtx.min_dist=distVandW;
						wVtx.pre_node=v;
						//[MUST] Remove and add w, o.w., elements in Q won't be in order
						Q.remove(wVtx);	//log|V|
						Q.add(wVtx);	//log|V|					
					}						
				}						
			}		
		}	

		return T;
	}

	/**
	 * Build a complete sub graph with start node and to-be-covered nodes based on route network
	 * @param start_node start node
	 * @param route_network route network
	 * @param cover_nodes covered nodes
	 * @return Graph object
	 */
	public static <V> Graph<V> buildCompleteGraph	(V start_node, Graph<V> route_network, HashSet<V> cover_nodes){
		Graph<V> G = new Graph<V>(true);		

		HashSet <V> nodeSet=new HashSet <V> (cover_nodes);
		nodeSet.add(start_node);	
		for(V v: nodeSet){
			//System.out.println(v);
			WrappedObject <V> shortestPaths=dijkstraShortestPath(route_network, v);			
			for(V w: nodeSet){
				//System.out.println(w);
				if(w.equals(v)) continue;
				G.addArc(v, w, shortestPaths.dist.get(w));
			}
		}		
		return G;
	}

	/**
	 * Single source Dijkstra algorithm
	 * @param graph graph
	 * @param source source vertex
	 * @return list of shortest distance and previous node
	 */
	public static <V> WrappedObject <V> dijkstraShortestPath (Graph<V> graph,V source) {
		/*System.out.println("Shortest Path:"+source+"->ALL");*/
		HashMap<V, Double> dist = new HashMap<V, Double>(); //distance to all vertices
		HashMap<V, V> prev = new HashMap<V, V>(); //predecessor list
		PriorityQueue<Vertex<V>> Q = new PriorityQueue<>();
		graph.getVertexList().get(source).min_dist=0;
		dist.put(source, 0.0);
		graph.getVertexList().get(source).pre_node=null;
		prev.put(source, null);

		//Initialization
		for (V v: graph.getVertexList().keySet()){
			if (!v.equals(source)) {
				graph.getVertexList().get(v).min_dist=Double.POSITIVE_INFINITY;
				dist.put(v, Double.POSITIVE_INFINITY);				

				//prev.put(v, null);
				graph.getVertexList().get(v).pre_node=null;
				prev.put(v, null);

			}
			Q.offer(graph.getVertexList().get(v));			
		}	

		//Main Procedure
		while (!Q.isEmpty()){
			Vertex <V> u=Q.poll();

			for (V v : graph.getAdjacentVertices((V) u.getID())){
				//double alt = dist.get(u) + graph.getDistanceBetween(u, v);
				double alt = u.min_dist + graph.getDistanceBetween((V) u.getID(), v);
				if (alt<graph.getVertexList().get(v).min_dist){
					Q.remove(graph.getVertexList().get(v));	

					graph.getVertexList().get(v).min_dist=alt;
					dist.put(v, alt);											

					graph.getVertexList().get(v).pre_node=u.getID();
					prev.put(v, (V) u.getID());

					Q.add(graph.getVertexList().get(v));					
				}				
			}
		}

		/*System.out.println("Dist:"+dist);*/
		return new WrappedObject(dist, prev);
	}

	//	public static <V> WrapWalkWithLength <V> shortestCoverCostGreedy(V start_node, Graph<V> graph, HashSet<V> cover_nodes){
	//		//MAKE a copy of nodes to be covered
	//		HashSet<V> candidate_nodes=new HashSet<V>(cover_nodes);
	//		candidate_nodes.remove(start_node); //Except the start
	//
	//		//Current node
	//		V cur_node=start_node;
	//
	//		//Approximate shortest walk
	//		ArrayList <V> walk = new ArrayList <V> ();
	//
	//		//Approximate shortest walk distance
	//		double walk_len=0;
	//
	//		//Keep adding nearest neighbor until all nodes are covered
	//		while(!candidate_nodes.isEmpty()){		
	//			//COMPUTE shortest to all other nodes by Dijkstra algorithm
	//			WrappedObject <V> reObjs=dijkstraShortestPath(graph, cur_node);
	//			double shortest_distance=Double.POSITIVE_INFINITY;	
	//			V nex_nn=null; //next nearest neighbor
	//
	//			for (V v: candidate_nodes){
	//				//System.out.println(reObjs.dist.get(v));
	//
	//				if (reObjs.dist.get(v)<=shortest_distance){
	//					shortest_distance=reObjs.dist.get(v);
	//					//System.out.println(shortest_distance);
	//					nex_nn=v;					
	//				}
	//			}
	//
	//			candidate_nodes.remove(nex_nn);
	//
	//			walk_len+=shortest_distance;
	//
	//			//RECOVER the new walk
	//			ArrayList <V> walk_append = new ArrayList <V>();			
	//			V u=nex_nn;
	//			walk_append.add(0, u);
	//
	//			while(reObjs.prev.get(u)!=null){
	//				walk_append.add(0, reObjs.prev.get(u)); //Add to the head of the list
	//				u=reObjs.prev.get(u);			
	//			}
	//
	//
	//			//APPEND the new walk
	//			if (walk.size()>0) walk.remove(walk.size()-1);
	//			walk.addAll(walk_append);
	//
	//			cur_node=nex_nn;
	//		}
	//
	//		//RETURN to starting node
	//		WrappedObject <V> reObjs_back=dijkstraShortestPath(graph, cur_node);
	//		walk_len+=reObjs_back.dist.get(start_node);
	//
	//		//RECOVER walk back to home
	//		ArrayList <V> walk_append_back = new ArrayList <V>();			
	//		V u=start_node;
	//		walk_append_back.add(0, u);
	//
	//		while(reObjs_back.prev.get(u)!=null){
	//			walk_append_back.add(0, reObjs_back.prev.get(u)); //Add to the head of the list
	//			u=reObjs_back.prev.get(u);			
	//		}			
	//
	//		//APPEND walk back to home
	//		if (walk.size()>0) walk.remove(walk.size()-1);
	//		walk.addAll(walk_append_back);	
	//
	//		return new WrapWalkWithLength(walk, walk_len);		
	//	}


	public static <V> WrapWalkWithLength <V> shortestCoverCostGreedy(V start_node, Graph<V> graph, HashSet<V> cover_nodes){
		//Approximate shortest walk
		ArrayList <V> walk = new ArrayList <V> ();

		//Build a complete subgraph G from route network which includes start vertex and covering nodes
		Graph<V> G =buildCompleteGraph(start_node, graph, cover_nodes);				
		//System.out.println("Complete graph G:"+G.toString());

		//MAKE a copy of nodes to be covered
		HashSet<V> candidate_nodes=new HashSet<V>(G.vertexList.keySet());
		candidate_nodes.remove(start_node); //Except the start
		walk.add(start_node);

		//Current node
		V cur_node=start_node;

		//Approximate shortest walk distance
		double walk_len=0;

		//Keep adding nearest neighbor until all nodes are covered
		while(!candidate_nodes.isEmpty()){		
			//COMPUTE shortest to all other nodes by Dijkstra algorithm
			//WrappedObject <V> reObjs=dijkstraShortestPath(graph, cur_node);
			double shortest_distance=Double.POSITIVE_INFINITY;	
			V nex_nn=null; //next nearest neighbor

			for (V v: candidate_nodes){
				double dist=G.getDistanceBetween(cur_node, v);
				//System.out.println(reObjs.dist.get(v));
				if (dist<shortest_distance){
					shortest_distance=dist;
					//System.out.println(shortest_distance);
					nex_nn=v;					
				}
			}

			if(nex_nn!=null){
				walk.add(nex_nn);
				candidate_nodes.remove(nex_nn);
				walk_len+=shortest_distance;
				cur_node=nex_nn;
			}else {//Having unreachable node
				//System.out.println("unreach:"+candidate_nodes);
				walk.clear();
				return new WrapWalkWithLength(walk, Double.POSITIVE_INFINITY);
			}
		}

		//RETURN to starting node
		walk_len+=G.getDistanceBetween(cur_node, start_node);
		walk.add(start_node);

		return new WrapWalkWithLength(walk, walk_len);		
	}
	
	
	/**
	 * This method implements Dijkstra algorithm with source and target node
	 * @param graph graph
	 * @param source source node
	 * @param target target node
	 * @return the shortest path
	 */
	public static <V> WrapWalkWithLength dijkstraShortestPath (Graph<V> graph,V source, V target) {
		//System.out.println("Shortest Path:"+source+"->"+target);
		ArrayList <V> shortest_path= new ArrayList <V> ();

		//Special case: source == target
		if(source.equals(target)) {
			shortest_path.add(0, target);
			shortest_path.add(0, source);
			return new WrapWalkWithLength(shortest_path, 0);				
		}


		//HashMap<V, Double> dist = new HashMap<V, Double>(); //distance to all vertices
		HashMap<V, V> prev = new HashMap<V, V>(); //predecessor list

		PriorityQueue<Vertex<V>> Q = new PriorityQueue<>();


		graph.getVertexList().get(source).min_dist=0;		
		graph.getVertexList().get(source).pre_node=null;
		prev.put(source, null);

		//Initialization
		for (V v: graph.getVertexList().keySet()){
			if (!v.equals(source)) {
				//dist.put(v, Double.POSITIVE_INFINITY);
				graph.getVertexList().get(v).min_dist=Double.POSITIVE_INFINITY;

				//prev.put(v, null);
				graph.getVertexList().get(v).pre_node=null;
				prev.put(v, null);

			}
			Q.offer(graph.getVertexList().get(v));			
		}	

		//Main Procedure
		while (!Q.isEmpty()){
			Vertex u=Q.poll();

			//Terminate if currently-considered vertex is the target
			if(u.getID().equals(target)) break;


			for (V v : graph.getAdjacentVertices((V) u.getID())){
				//double alt = dist.get(u) + graph.getDistanceBetween(u, v);
				double alt = u.min_dist + graph.getDistanceBetween((V) u.getID(), v);
				if (alt<graph.getVertexList().get(v).min_dist){
					Q.remove(graph.getVertexList().get(v));					
					graph.getVertexList().get(v).min_dist=alt;					
					graph.getVertexList().get(v).pre_node=u.getID();
					prev.put(v, (V) u.getID());

					Q.add(graph.getVertexList().get(v));					
				}				
			}
		}

		//RETRIVE path and distance
		V u=target;
		shortest_path.add(0, u);
		double length=0;

		while(graph.getVertexList().get(u).pre_node!=null){
			shortest_path.add(0, (V) graph.getVertexList().get(u).pre_node); //Add to the head of the list
			length+=graph.getDistanceBetween((V) graph.getVertexList().get(u).pre_node, u);
			u=(V) graph.getVertexList().get(u).pre_node;			
		}	

		WrapWalkWithLength res=new WrapWalkWithLength(shortest_path, length);
		return res;
	}
}