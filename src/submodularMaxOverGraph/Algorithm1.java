package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;

public class Algorithm1 {

	public Algorithm1() {
		// TODO Auto-generated constructor stub
	}
	
	
//	public static <V> HashMap<V, Double> dijkstraShortestPath(Graph<V> graph,V source) {
	//		HashMap<V, Double> distances = new HashMap<V, Double>();
	//		ArrayList<V> queue = new ArrayList<V>();
	//		ArrayList<V> visited = new ArrayList<V>();
	//		queue.add(0, source);
	//		distances.put(source, 0.0);
	//		while (!queue.isEmpty()) {
	//			V currentVertex = queue.remove(queue.size() - 1);
	//			// to save time we initialize all the distances to infinity as we go
	//			if (distances.get(currentVertex) == null) {
	//				distances.put(currentVertex, Double.POSITIVE_INFINITY);
	//			}
	//			for (V adjacentVertex : graph.getAdjacentVertices(currentVertex)) {
	//				if (distances.get(adjacentVertex) == null) {
	//					distances.put(adjacentVertex, Double.POSITIVE_INFINITY);
	//				}
	//				// if the distance between the source and the adjacent vertex is
	//				// greater than the distance between the source and the current
	//				// vertex PLUS the weight between the current and adjacent
	//				// vertex, then we have found a shorter path than already
	//				// existed
	//				if (true) {
	//					if (distances.get(adjacentVertex) > graph.getDistanceBetween(currentVertex, adjacentVertex)+ distances.get(currentVertex)) {
	//						distances.put(
	//								adjacentVertex,
	//								graph.getDistanceBetween(currentVertex,adjacentVertex)+ distances.get(currentVertex));
	//					}
	//				}
	//				
	//				if (!visited.contains(adjacentVertex)&&!queue.contains(adjacentVertex)) {
	//					queue.add(0, adjacentVertex);
	//				}
	//			}
	//			visited.add(currentVertex);
	//		}
	//		// since the above statments only added the vertices as needed,
	//		// verticies that are completely unconnected to the source are not added
	//		// yet, so this adds them now
	//		for (V v : graph.getVertexList().keySet()) {
	//			if (!distances.containsKey(v)) {
	//				distances.put(v, Double.POSITIVE_INFINITY);
	//			}
	//		}
	//		return distances;
	//	}


	//This method implement Dijkstra shortest path algorithm, i.e. min-priority queue and returns a path from source to target
	//Reference: http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm (SAME alphabets)
	//http://en.literateprograms.org/index.php?title=Special:DownloadCode/Dijkstra%27s_algorithm_%28Java%29&oldid=15444
	//https://www.youtube.com/watch?v=gdmfOwyQlcI

	public static <V> ArrayList<V> dijkstraShortestPath (Graph<V> graph,V source, V target) {
		System.out.println("Shortest Path:"+source+"->"+target);
		ArrayList <V> shortest_path= new ArrayList <V> ();

		//Special case: source == target
		if(source.equals(target)) {
			shortest_path.add(0, target);
			shortest_path.add(0, source);
			return shortest_path;				
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

		//RETRIVE path
		V u=target;
		shortest_path.add(0, u);

		while(graph.getVertexList().get(u).pre_node!=null){
			shortest_path.add(0, (V) graph.getVertexList().get(u).pre_node); //Add to the head of the list
			u=(V) graph.getVertexList().get(u).pre_node;			
		}	

		return shortest_path;
	}


	public static <V> ArrayList <V> recusiveGreedy(Graph<V> road_network, InfluenceModel<V> social_network, V start, V target, int budget, HashSet iniActiveNodes, int max_iter) {
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
		//path=dijkstraShortestPath(road_network, start, target);
		//Compute length of the shortest path
		//		double length=0;
		//		for(int i=0; i<path.size()-1; i++){
		//			length+=road_network.getDistanceBetween(path.get(i), path.get(i+1));			
		//		}

		//FORM a Path
		path.add(start);
		path.add(target);

		double length=road_network.getDistanceBetween(start, target);

		System.out.println(path+"="+length);	


		//Return infeasible if out of budget
		if(length>budget) {
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
			if (v.equals(target)||v.equals(start)) continue; //Finding a node between start and target

			//if(tracker.contains(v)) continue;

			for (int b=1; b<=budget; b++){
				System.out.println("b="+b);
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
					}				
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

}
