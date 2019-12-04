package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;
/**
 * This class implements Iterated Sub-modular Cost Knapsack (ISK, Iyer 2013)
 * @author zhangh24
 *
 */
public class Algorithm6 {

	public Algorithm6() {
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


	public static <V> ArrayList <V> greedyWalk(int mode, Graph<V> road_network, InfluenceModel<V> social_network, 
			V start, V target, double budget, HashSet <V> iniActiveNodes) {

		HashSet<V> X_t=new HashSet<V>(); //Solution obtained from iteration t
		ArrayList <V> walk_best = null;  //Best walk

		for (int t=0; t<((mode==1)?Driver_dtd.maxIterISK:Driver_rg.maxIterISK); t++){
			//At each iteration solve a sub-modular maximization problem s.t. knapsack constraint

			//RESET influence value tracker 
			Driver_dtd.max_influe=0;
			Driver_rg.max_influe=0;//[BUG]

			//DEFINED two lists: one for visited set, one for unvisited set
			HashSet<V> visited_set=new HashSet<V>();
			HashSet<V> unvisited_set=new HashSet<V>();

			//ADD all social nodes to unvisited set if not initially visited
			for (V v:social_network.getSocialNetWork().getVertexList().keySet()){
				if(iniActiveNodes.contains(v)) visited_set.add(v);
				else unvisited_set.add(v);
			}

			//BEST walk tracked so far
			//ArrayList <V> walk_best = null;
			
			double infl_cur=social_network.getExpectedInfluence(visited_set); //influence before choosing a new vertex			
			
			//PICK the node in the unvisited set with max marginal influence per cost and ADD to the visited set
			while (!unvisited_set.isEmpty()){
				V node_best_iter = null;  //BEST node
				ArrayList <V> walk_best_iter = new ArrayList <V>(); //BEST walk

				double fc_max_iter=0; //MAX influence
				//double delta_f_max=0; //MAX influence change
				double cost_best_iter=0; //BEST cost by adding BEST node

				//double infl_cur=social_network.getExpectedInfluence(visited_set); //influence before choosing a new vertex			

				for(V v:unvisited_set){	
					//System.out.println("v="+v+","+"#"+ln--);					
					HashSet <V> u=union(v, visited_set);					

					//GET upper bound value of cost function
					HashSet<V> new_set=new HashSet<V> ();
					new_set.add(v);
					double cost_new=getUpperBoundVal(mode, X_t, new_set, start,road_network);//c(j)

					HashSet<V> new_set_union=union(new_set, exclude(visited_set, iniActiveNodes));
					double cost_new_accu=getUpperBoundVal(mode, X_t, new_set_union, start, road_network);//c(X_t \cup j)

					//OPTIMIZE PERFORMANCE: method 1
					//SKIP infeasible nodes to boost speed
					if(Driver_dtd.prune_mode||Driver_rg.prune_mode){
						if(cost_new_accu>budget) continue;					
					}

					//Influence
					double infl_new=social_network.getExpectedInfluence(u); //influence when adding the new vertex
					double delta_f=infl_new-infl_cur; //influence changes

					//System.out.println("delta_f="+delta_f);

					double fc=delta_f/cost_new;//gradient
					//System.out.println("fc="+fc);

					if (fc>=fc_max_iter){
						//System.out.println("better than: "+fc_max);
						infl_cur=infl_new;
						node_best_iter=v;
						fc_max_iter=fc;					
						//delta_f_max=delta_f;					
						//walk_best_iter=walk_new.walk;
						cost_best_iter=cost_new_accu;									
					}else{
						//System.out.println("worse than:"+fc_max);
					}
				}			


				if (node_best_iter!=null&&cost_best_iter<=budget){//method 0
					//if (node_best_iter!=null){ //method 1
					walk_best=walk_best_iter;
					visited_set.add(node_best_iter); //ADD the node to visited set
					unvisited_set.remove(node_best_iter); //REMOVE the node from unvisited set
					//System.out.println("visited set:"+visited_set+";cost:"+cost_new_max+";walk:"+walk_new_max);
					if(mode==1){
						//Driver.max_influe+=delta_f_max;
						Driver_dtd.budget=cost_best_iter;//Track budget usage
						Driver_dtd.visit_set=exclude(visited_set, iniActiveNodes);//Not include initially activited nodes
					}else if(mode==2){
						//Driver_rg.max_influe+=delta_f_max;
						Driver_rg.budget=cost_best_iter;//Track budget usage
						Driver_rg.visit_set=exclude(visited_set, iniActiveNodes);//Not include initially activited nodes
					}

				}else break;			
			}

			//UPDATE X_t
			X_t=exclude(visited_set,iniActiveNodes);
			//System.out.println(X_t);
			//Driver.max_influe+=(social_network.getExpectedInfluence(visited_set)-base_influence);
			
			//Adjusted Influence[jair]
			double influAdj=(visited_set.size()==iniActiveNodes.size())?0:
				social_network.getExpectedInfluence(visited_set)-social_network.getExpectedInfluence(iniActiveNodes);
			if(mode==1) Driver_dtd.max_influe=influAdj;
			else if(mode==2){
				Driver_rg.max_influe=influAdj;
			}
		}

		//END iteration i
		WrapWalkWithLength walk_best_with_cost=shortestWalkCoverCost(start,road_network,X_t);
		walk_best=walk_best_with_cost.walk;

		return walk_best;		
	}

	public static <V> ArrayList<V> greedyWalk_sp (int mode, Graph<V> sensorNet,
			DoubleMatrix cov, V start,
			V target, double budget, HashSet<V> s) {
		HashSet<V> X_t=new HashSet<V>(); //Solution obtained from iteration t
		ArrayList <V> walk_best = new ArrayList <V>();  //Best walk

		for (int t=0; t<Driver_sp.maxIterISK; t++){
			//At each iteration solve a sub-modular maximization problem s.t. knapsack constraint

			//RESET influence value tracker
			Driver_sp.max_entropy=0;

			//DEFINED two lists: one for visited set, one for unvisited set
			HashSet<V> visited_set=new HashSet<V>();
			HashSet<V> unvisited_set=new HashSet<V>();

			//ADD all social nodes to unvisited set if not initially visited
			for (V v:sensorNet.getVertexList().keySet()){
				if(s.contains(v)) visited_set.add(v);
				else unvisited_set.add(v);
			}

			//BEST walk tracked so far
			//ArrayList <V> walk_best = null;

			//PICK the node in the unvisited set with max marginal influence per cost and ADD to the visited set
			while (!unvisited_set.isEmpty()){
				V node_best_iter = null;  //BEST node
				ArrayList <V> walk_best_iter = new ArrayList <V>(); //BEST walk

				double fc_max_iter=0; //MAX influence
				double delta_f_max=0; //MAX influence change
				double cost_best_iter=0; //BEST cost by adding BEST node

				//double infl_cur=sensorNet.getExpectedInfluence(visited_set); //influence before choosing a new vertex			

				for(V v:unvisited_set){	
					//System.out.println("v="+v+","+"#"+ln--);					
					HashSet <V> u=union(v, visited_set);					

					//GET upper bound value of cost function
					HashSet<V> new_set=new HashSet<V> ();
					new_set.add(v);
					double cost_new=getUpperBoundVal(mode, X_t, new_set, start, sensorNet);//c(j)

					HashSet<V> new_set_union=union(new_set, exclude(visited_set, s));
					double cost_new_accu=getUpperBoundVal(mode, X_t, new_set_union, start,sensorNet);//c(X_t \cup j)

					//OPTIMIZE PERFORMANCE: method 1
					//SKIP infeasible nodes to boost speed
					if(Driver_sp.prune_mode){
						if(cost_new_accu>budget) continue;		
					}

					//Conditional Entropy/Variance
					double K_yy=subMatrix(cov, v, v).get(0,0); 
					DoubleMatrix K_yA=(visited_set.size()>0)?subMatrix(cov, v, visited_set):new DoubleMatrix(1,1,0);
					DoubleMatrix K_AA_ivs=(visited_set.size()>0)?Solve.pinv(subMatrix(cov, visited_set, visited_set)):new DoubleMatrix(1,1,0);
					double delta_f=K_yy-
							(K_yA.mmul(K_AA_ivs).mmul(K_yA.transpose())).get(0,0); //utility changes

					//Convert to entropy instead of conditional variance
					delta_f=0.5*Math.log(2*Math.PI*Math.E*delta_f);

					//System.out.println("delta_f="+delta_f);

					double fc=delta_f/cost_new;//gradient
					//System.out.println("fc="+fc);

					if (fc>=fc_max_iter){
						//System.out.println("better than: "+fc_max);
						node_best_iter=v;
						fc_max_iter=fc;					
						delta_f_max=delta_f;					
						//walk_best_iter=walk_new.walk;
						cost_best_iter=cost_new_accu;									
					}else{
						//System.out.println("worse than:"+fc_max);
					}
				}			


				if (node_best_iter!=null&&cost_best_iter<=budget){//method 0
					//if (node_best_iter!=null){ //method 1
					walk_best=walk_best_iter;
					visited_set.add(node_best_iter); //ADD the node to visited set
					unvisited_set.remove(node_best_iter); //REMOVE the node from unvisited set
					//System.out.println("visited set:"+visited_set+";cost:"+cost_new_max+";walk:"+walk_new_max);
					Driver_sp.budget=cost_best_iter;//Track budget usage
					Driver_sp.max_entropy+=delta_f_max;
					//Driver_sp.visit_set=visited_set;
					Driver_sp.visit_set=exclude(visited_set, s); //fixed logging bug [HZ]

				}else break;			
			}

			//UPDATE X_t
			X_t=exclude(visited_set,s);
			//Driver.max_influe+=(social_network.getExpectedInfluence(visited_set)-base_influence);	

		}

		//END iteration i
		WrapWalkWithLength walk_best_with_cost=shortestWalkCoverCost(start,sensorNet,X_t);
		walk_best=walk_best_with_cost.walk;


		return walk_best;		
	}

	public static <V> double getUpperBoundVal(int mode, HashSet <V> X_t, HashSet <V> X, V start,  Graph<V> road_network){
		//c(X_t)
		double v1= shortestWalkCoverCost(start,road_network,X_t).length;
		if(mode==1) v1+=Driver_dtd.visit_cost*X_t.size();
		else if(mode==2) v1+=Driver_rg.visit_cost*X_t.size();
		else if(mode==3) v1+=Driver_sp.visit_cost*X_t.size();


		//\sum_{j\in {X_t}\backslash X}c(j|X_t\backslash j)
		double v2=0	;
		HashSet <V> X_t_no_X=exclude(X_t, X); // Elements in X_t but not in X 	

		if(X_t_no_X.size()>0) {		
			for(V v: X_t_no_X ){
				HashSet <V> j=new HashSet();
				j.add(v);

				HashSet <V> X_t_no_j=exclude(X_t, j);			

				if(mode==1)
					v2+=((shortestWalkCoverCost(start,road_network,X_t).length+Driver_dtd.visit_cost*X_t.size())
							-(shortestWalkCoverCost(start,road_network,X_t_no_j).length+Driver_dtd.visit_cost*X_t_no_j.size()));
				else if (mode==2)
					v2+=((shortestWalkCoverCost(start,road_network,X_t).length+Driver_rg.visit_cost*X_t.size())
							-(shortestWalkCoverCost(start,road_network,X_t_no_j).length+Driver_rg.visit_cost*X_t_no_j.size()));
				else if (mode==3)
					v2+=((shortestWalkCoverCost(start,road_network,X_t).length+Driver_sp.visit_cost*X_t.size())
							-(shortestWalkCoverCost(start,road_network,X_t_no_j).length+Driver_sp.visit_cost*X_t_no_j.size()));

			}
		}

		//\sum_{j\in {X}\backslash X_t}c(j|\emptyset)		
		double v3=0;

		HashSet <V> X_no_X_t=exclude(X, X_t); // Elements in X_t but not in X 	

		if(X_no_X_t.size()>0) {		
			for(V v: X_no_X_t ){
				HashSet <V> j=new HashSet();
				j.add(v);

				if(mode==1)
					v3+=(shortestWalkCoverCost(start,road_network,j).length+Driver_dtd.visit_cost*j.size());
				else if(mode==2)
					v3+=(shortestWalkCoverCost(start,road_network,j).length+Driver_rg.visit_cost*j.size());
				else if(mode==3)
					v3+=(shortestWalkCoverCost(start,road_network,j).length+Driver_sp.visit_cost*j.size());
			}
		}			

		return v1-v2+v3;
	}


	public static <V> WrapWalkWithLength shortestWalkCoverCost(V start_node, Graph<V> route_network, HashSet<V> cover_nodes){
		//MAKE a copy of nodes to be covered
		HashSet<V> candidate_nodes=new HashSet<V>(cover_nodes);
		//candidate_nodes.remove(start_node); //Except the start

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

	//	public static <V> HashSet union(ArrayList <V> path, HashSet active_nodes){
	//		HashSet <V> union=new HashSet <V> (active_nodes);		
	//		for (V v: path){
	//			union.add(v);		
	//		}
	//		return union;		
	//	}

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
