package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Random;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;
/**
 * This class implements method to estimate k_f, gamma, beta and alpha.
 * @author zhangh24
 * @param <V>
 *
 */
public class Algorithm7<V> {

	public Algorithm7() {
		// TODO Auto-generated constructor stub
	}

	/**
	 * Single source Dijkstra algorithm
	 * @param graph graph
	 * @param source source vertex
	 * @return a list of distance from source and preceding vertex for each vertex
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

		/*System.out.println("Dist:"+dist);*/
		return new WrappedObject(dist, prev);
	}
	/**
	 * Estimate curvature as well as gamma and beta.
	 * @param road_network routing networks
	 * @param social_network influence networks
	 * @param start start node
	 * @param visit_cost visit cost per node
	 * @return
	 */
	public static <V> double estimateCurvature(Graph<V> road_network, InfluenceModel<V> social_network,V start, double visit_cost) {		
		double gamma=Double.POSITIVE_INFINITY; //min f(j|S\j)/f(j)
		double beta=Double.NEGATIVE_INFINITY; //max f(j|S\j)/f(j)

		double visit_cost_per_node=visit_cost; //Test Graph: >=2

		double k_f=0; //curvature: 1-gama

		//Set of all nodes in road networks
		HashSet<V> E=new HashSet<V>();

		//TEST Reachablity
		WrappedObject <V> reObjs=dijkstraShortestPath(road_network, start);

		//ADD all social nodes to unvisited set if not initially visited
		for (V v:social_network.getSocialNetWork().getVertexList().keySet()){
			if(reObjs.dist.get(v)!=Double.POSITIVE_INFINITY) //ADD ONLY node which is reachable
				E.add(v);
		}		

		WrapWalkWithLength f_E_walk=shortestWalkCoverCost(start,road_network,E);
		double f_E=f_E_walk.length+E.size()*visit_cost_per_node;//TRY to add visit cost, so that a monotone cost

		for(V v: E){
			HashSet <V> set_j=new HashSet();
			set_j.add(v);
			WrapWalkWithLength f_E_no_j_walk=shortestWalkCoverCost(start,road_network,exclude(E,set_j));
			double f_E_no_j=f_E_no_j_walk.length+(E.size()-1)*visit_cost_per_node;//TRY to add visit cost, so that a monotone cost		
			double f_j=shortestWalkCoverCost(start,road_network,set_j).length+visit_cost_per_node;
			double ratio=(f_E-f_E_no_j)/f_j;
			//System.out.print(v+":"+ratio);//Print each vertex
			if(ratio<gamma) {
				//System.out.print(v+":"+ratio);
				//System.out.print(",Check");
				if(ratio<0) {
					//System.out.print(",OhNo");
					//System.out.println("Length:"+f_E+"Walk:"+f_E_walk.walk);
					//System.out.println("Length:"+f_E_no_j+"Walk:"+f_E_no_j_walk.walk);					
				}
				gamma=ratio;
				//System.out.println();
			}
			//System.out.println();

			if(ratio>beta) {
				beta=ratio;
			}			
		}

		System.out.print(gamma+",");
		System.out.print(beta+",");		
		System.out.println((1-gamma));		

		return 1-gamma;		
	}

	public static <V> double estimateAlpha(Graph<V> road_network, InfluenceModel<V> social_network,V start, double visit_cost, int iter_max) {
		double alpha=Double.POSITIVE_INFINITY;//alpha=min_x min_{A,B:A\subsetB} c(x|A)/c(x|B)

		//TEST Reachablity
		WrappedObject <V> reObjs=dijkstraShortestPath(road_network, start);

		//Conduct iter_max # of random draws		
		for(int i=0; i<iter_max; i++){
			HashSet<V> B=new HashSet<V>();
			HashSet<V> E_not_B=new HashSet<V>();
			HashSet<V> A=new HashSet<V>();
			V x = null;

			//Random generator
			Random rn = new Random(System.nanoTime());

			//ADD social nodes from E to set B randomly, i.e., p=0.5
			for (V v:social_network.getSocialNetWork().getVertexList().keySet()){
				double rn_db=rn.nextDouble();
				if(reObjs.dist.get(v)!=Double.POSITIVE_INFINITY){
					if(rn_db>0.5) B.add(v);//ADD ONLY node which is reachable
					else E_not_B.add(v);						
				}					
			}

			//ADD social nodes from B to set A randomly, i.e., p=0.5
			for (V v:B){
				double rn_db=rn.nextDouble();
				if(rn_db>0.5) //ADD ONLY node which is reachable
					A.add(v);
			}

			//PICK a random x from E-B
			int size = E_not_B.size();
			if(size>0){
				int item = rn.nextInt(size); // In real life, the Random object should be rather more shared than this
				int it = 0;
				for(V v : E_not_B)
				{
					if (it == item) x=v;
					it++;
				}

			}else continue;

			double c_xA=shortestWalkCoverCost(start,road_network,union(x,A)).length-shortestWalkCoverCost(start,road_network,A).length;
			//System.out.println("c_xA:");
			//System.out.println(shortestWalkCoverCost(start,road_network,union(x,A)).walk);
			//System.out.println(shortestWalkCoverCost(start,road_network,A).walk);
			//System.out.println(c_xA);

			double c_xB=shortestWalkCoverCost(start,road_network,union(x,B)).length-shortestWalkCoverCost(start,road_network,B).length;
			//System.out.println("c_xB:");
			//System.out.println(shortestWalkCoverCost(start,road_network,union(x,B)).walk);
			//System.out.println(shortestWalkCoverCost(start,road_network,B).walk);
			//System.out.println(c_xB);

			double ratio=c_xA/c_xB;
			//System.out.println("["+i+"]:"+ratio);
			//System.out.println("x:"+x);
			//System.out.println("A:"+A);
			//System.out.println("B:"+B);

			if(ratio>0&&ratio<alpha) {//ONLY select positive ratio
				//System.out.println("x:"+x);
				//System.out.println("A:"+A);
				//System.out.println("B:"+B);
				alpha=ratio;
				//System.out.println("Check!");
				//System.out.println("alpha:"+alpha);
			}

		}		

		System.out.println(alpha);
		return alpha;
	}	

	public static <V> WrapWalkWithLength shortestWalkCoverCost(V start_node, Graph<V> route_network, HashSet<V> cover_nodes){
		//MAKE a copy of nodes to be covered
		HashSet<V> candidate_nodes=new HashSet<V>(cover_nodes);

		//Current node
		V cur_node=start_node;

		//Approximate shortest walk
		ArrayList <V> walk = new ArrayList <V> ();

		//Approximate shortest walk distance
		double walk_len=0;

		//Keep adding nearest neighbor until all nodes are covered
		while(!candidate_nodes.isEmpty()){		
			//COMPUTE shortest to all other nodes by Dijkstra algorithm
			WrappedObject <V> reObjs=dijkstraShortestPath(route_network, cur_node);
			double shortest_distance=Double.POSITIVE_INFINITY;	
			V nex_nn=null; //next nearest neighbor

			for (V v: candidate_nodes){
				//System.out.println(reObjs.dist.get(v));
				if (reObjs.dist.get(v)<=shortest_distance){
					shortest_distance=reObjs.dist.get(v);
					//System.out.println(shortest_distance);
					nex_nn=v;					
				}
			}

			candidate_nodes.remove(nex_nn);

			walk_len+=shortest_distance;

			//RECOVER the new walk
			ArrayList <V> walk_append = new ArrayList <V>();			
			V u=nex_nn;
			walk_append.add(0, u);

			while(reObjs.prev.get(u)!=null){
				walk_append.add(0, reObjs.prev.get(u)); //Add to the head of the list
				u=reObjs.prev.get(u);			
			}			

			//APPEND the new walk
			if (walk.size()>0) walk.remove(walk.size()-1);
			walk.addAll(walk_append);

			cur_node=nex_nn;
		}

		//RETURN to starting node
		WrappedObject <V> reObjs_back=dijkstraShortestPath(route_network, cur_node);
		walk_len+=reObjs_back.dist.get(start_node);

		//RECOVER walk back to home
		ArrayList <V> walk_append_back = new ArrayList <V>();			
		V u=start_node;
		walk_append_back.add(0, u);

		while(reObjs_back.prev.get(u)!=null){
			walk_append_back.add(0, reObjs_back.prev.get(u)); //Add to the head of the list
			u=reObjs_back.prev.get(u);			
		}			

		//APPEND walk back to home
		if (walk.size()>0) walk.remove(walk.size()-1);
		walk.addAll(walk_append_back);	

		return new WrapWalkWithLength(walk, walk_len);		
	}

	public static <V> HashSet union(V new_node, HashSet <V> active_nodes){
		HashSet <V> union=new HashSet <V> (active_nodes);		
		union.add(new_node);
		return union;		
	}

	public static <V> HashSet union( HashSet <V> node_set_1, HashSet <V> node_set_2){
		HashSet <V> union=new HashSet <V> (node_set_1);
		//		for (V v: node_set_2){
		//			union.add(v);		
		//		}
		union.addAll(node_set_2);
		return union;			
	}	

	public static <V> HashSet<V> exclude( HashSet <V> node_set_1, V no_node){
		HashSet<V> node_set_2=new HashSet <V> ();
		node_set_2.add(no_node);
		return exclude(node_set_1, node_set_2);

	}


	public static <V> HashSet<V> exclude( HashSet <V> node_set_1, HashSet <V> node_set_2){
		HashSet <V> exclude=new HashSet <V> ();
		for(V v: node_set_1){
			if(!node_set_2.contains(v)) exclude.add(v);  
		}		
		return exclude;			
	}

	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, HashSet<V> list_row, HashSet<V> list_col) {
		DoubleMatrix sub_matrix = null;
		int[] list_row_num=new int[list_row.size()];
		int[] list_col_num=new int[list_col.size()];				

		int i=0;

		for(V v: list_row){
			int idx=Integer.valueOf((String) v)-1001;
			list_row_num[i]=idx;
			i++;
		}

		i=0;
		for(V v: list_col){
			int idx=Integer.valueOf((String) v)-1001;
			list_col_num[i]=idx;
			i++;
		}

		//1 GET rows
		sub_matrix=matrix.getRows(list_row_num);				
		//2 GET cols	
		sub_matrix=sub_matrix.getColumns(list_col_num);

		return sub_matrix;		
	}

	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, V v1, V v2) {
		HashSet s_v1=new HashSet();
		HashSet s_v2=new HashSet();

		s_v1.add(v1);
		s_v2.add(v2);

		return subMatrix(matrix, s_v1, s_v2);		
	}

	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, V v, HashSet<V> list_col) {
		HashSet s_v=new HashSet();
		s_v.add(v);
		return subMatrix(matrix, s_v, list_col);

	}


}
