package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;

public class Algorithm2 {
	//Use single source Dijkstra algorithm, which computes shortest path to every other node
	public static <V> WrappedObject <V> dijkstraShortestPath (Graph<V> graph,V source) {
		System.out.println("Shortest Path:"+source+"->ALL");

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
			Vertex u=Q.poll();

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

		System.out.println("Dist:"+dist);
		return new WrappedObject(dist, prev);
	}

	public static <V> ArrayList <V> recusiveGreedy(Graph<V> road_network, InfluenceModel<V> social_network, V start, V target, double budget, HashSet iniActiveNodes, int max_iter) {
		System.out.println(">>>ENTER RG:"+start+","+target+","+budget+","+max_iter);

		//		System.out.println("Stack Push:"+target);
		//		if(!tracker.contains(target))
		//			tracker.push(target);
		//		System.out.println("Stack:"+tracker);

		//		if(start.equals("A")&&target.equals("E")&&budget==9){
		//			System.out.println("AE-7");	
		//		}


		double max_influence=0;
		ArrayList <V> path=new ArrayList <V> (); //Path thru which to seed

		//GET path by Dijkstra algorithm
		//WrappedObject <V> reObjs=dijkstraShortestPath(road_network, start);
		//path=dijkstraShortestPath(road_network, start, target);
		//RETRIVE path
		//		V u=target;
		//		path.add(0, u);
		//
		//		while(reObjs.prev.get(u)!=null){
		//			path.add(0, reObjs.prev.get(u)); //Add to the head of the list
		//			u=reObjs.prev.get(u);			
		//		}	
		//
		//		System.out.println(path+"="+reObjs.dist.get(target));	

		//FORM a Path
		path.add(start);
		path.add(target);

		double length=road_network.getDistanceBetween(start, target);
		System.out.println(path+"="+length);	


		//Return infeasible if out of budget
		//if(reObjs.dist.get(target)>budget) {
		if(length>budget){
			System.out.println("<<<RETURN RG:infeasible");
			//System.out.println("Stack Pop:"+target);
			//tracker.pop();
			return null;
		}

		//Base case
		if (max_iter==0) {
			//System.out.println("Stack Pop:"+target);
			System.out.println("<<<RETURN RG:zero iter");
			//tracker.pop();			
			return path;
		}

		//Benchmark influence
		double bmark_influence=social_network.getExpectedInfluence(union(path, iniActiveNodes));
		//Baseline influence
		double base_influence=social_network.getExpectedInfluence(iniActiveNodes);

		System.out.println("bmark_influence:"+bmark_influence);
		System.out.println("[SEND] base_influence:"+base_influence);


		max_influence=bmark_influence-base_influence;


		//Enforce to be zero, since both are the same.
		if(union(path, iniActiveNodes).equals(iniActiveNodes)) {
			max_influence=0;
			System.out.println("Benchmark Delta:"+max_influence);
		}

		if (max_influence<0){
			System.out.println("Benchmark Delta:"+max_influence);	
		}

		ArrayList <V> path_1 = new ArrayList <V> ();
		ArrayList <V> path_2 = new ArrayList <V> ();


		for (V v: road_network.getVertexList().keySet()){
			//if (v.equals(target)) continue; //Attempting to escape infinite loop
			if (v.equals(target)||v.equals(start)) continue; //Attempting to escape infinite loop

			//if(tracker.contains(v)) continue;

			for (double b=1; b<=budget; b++){
				System.out.println("b="+b);
				//if (b<reObjs.dist.get(v)) continue;  //PRUNING infeasible middle nodes	
				if (b<road_network.getDistanceBetween(start, v)) {
					System.out.println(">>>SKIP RG:"+start+","+v+","+b+","+(max_iter-1));
					continue;
				}
				
				path_1 = recusiveGreedy(road_network, social_network, start, v, b, iniActiveNodes, max_iter-1);
				System.out.println("Path_1:"+path_1);
				if (path_1!=null) {				
					path_2 = recusiveGreedy(road_network, social_network, v, target, budget-b, union(path_1,iniActiveNodes), max_iter-1);
					System.out.println("Path_2:"+path_2);

					if(path_2!=null){
						path_2.remove(0);

						path_1.addAll(path_2);

						double new_influence=social_network.getExpectedInfluence(union(path_1, iniActiveNodes));

						System.out.println("new_influence:"+new_influence);
						System.out.println("[ECHO] base_influence:"+base_influence);

						double delta_new=new_influence-base_influence;

						if(delta_new>max_influence)
						{
							path=path_1;
							max_influence=delta_new;
							System.out.println("Better Path");
						}	
					} else break; //STOP process further for smaller budgets, since path_2 is already infeasible.
				}
			}		
		}		

		//System.out.println("Stack Pop:"+target);
		//tracker.pop();

		System.out.println("<<<RETURN RG:non zero iter");
		System.out.println("MAX_INFLUENCE:"+max_influence);
		System.out.println("PATH_FOUND:"+path);
		return path;		
	}

	public static <V> HashSet union(ArrayList <V> path, HashSet active_nodes){
		HashSet <V> union=new HashSet <V> (active_nodes);		
		for (V v: path){
			union.add(v);		
		}
		return union;		
	}


	public static <V> void breadthFirstSearch(Graph<V> graph, V start) {
		ArrayList<V> queue = new ArrayList<V>();
		ArrayList<V> visited = new ArrayList<V>();
		queue.add(0, start);
		while (!queue.isEmpty()) {
			V currentVertex = queue.remove(queue.size() - 1);
			System.out.println(currentVertex);
			for (V adjacentVertex : graph.getAdjacentVertices(currentVertex)) {
				System.out.println("Adjacent to " + currentVertex + " : "
						+ adjacentVertex);
				if (!visited.contains(adjacentVertex)
						&& !queue.contains(adjacentVertex)) {
					queue.add(0, adjacentVertex);
				}
			}
			visited.add(currentVertex);
		}
	}


	public static <V> void depthFirstSearch(Graph<V> graph, V start) {
		ArrayList<V> queue = new ArrayList<V>();
		ArrayList<V> visited = new ArrayList<V>();
		queue.add(0, start);
		while (!queue.isEmpty()) {
			V currentVertex = queue.remove(0);
			System.out.println(currentVertex);
			for (V adjacentVertex : graph.getAdjacentVertices(currentVertex)) {
				System.out.println("Adjacent to " + currentVertex + " : "
						+ adjacentVertex);
				if (!visited.contains(adjacentVertex)
						&& !queue.contains(adjacentVertex)) {
					queue.add(0, adjacentVertex);
				}
			}
			visited.add(currentVertex);
		}
	}

}

