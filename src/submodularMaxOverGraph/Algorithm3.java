package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;
/**
 * This class implements a modified version of recursive greedy (Chekuri 05):
 * 1. Use single source Dijkstra algorithm to computes shortest path to every other node
 * 2. CONTINUE to next budget if current budget is less than shortest distance to middle node v
 * 3. BREAK budget loop if path v-t is infeasible
 * 4. Simple tweak: consider only nodes not on path s-t
 * @author zhangh24 *
 */
public class Algorithm3 {

	/**
	 * This method implements single source Dijkstra algorithm
	 * @param graph graph
	 * @param source source node
	 * @return an object which wraps distance and predecessor vectors 
	 */
	public static <V> WrappedObject <V> dijkstraShortestPath (Graph<V> graph,V source) {
		/*System.out.println("Shortest Path:"+source+"->ALL");*/

		HashMap<V, Double> dist = new HashMap<V, Double>(); //distance from source to all vertices
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


	public static <V> ArrayList <V> recusiveGreedy(int mode, Graph<V> road_network, InfluenceModel<V> social_network, V start, V target, int budget, HashSet <V> iniActiveNodes, int max_iter) {
		//System.out.println(">>>RG:"+start+","+target+","+budget+","+max_iter);
		//System.out.println("Depth:"+max_iter);

		double max_influence=0;
		ArrayList <V> path=new ArrayList <V> (); //Path thru which to seed

		//GET path by Dijkstra algorithm
		WrappedObject <V> reObjs=dijkstraShortestPath(road_network, start);

		//Return infeasible if out of budget
		double visitCostTarget=targetVisitCost(mode, social_network.socialNetwork, target, iniActiveNodes);	
		double costLB=(reObjs.dist.get(target)+visitCostTarget)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);

		if((int)costLB>(int)budget) { //path cost s-t and visit cost of node t
			return path;
		}

		//RETRIVE path (seems not need this!)
		//		V u=target;
		//		path.add(0, u);
		//
		//		while(reObjs.prev.get(u)!=null){
		//			path.add(0, reObjs.prev.get(u)); //Add to the head of the list
		//			u=reObjs.prev.get(u);			
		//		}	


		path.add(0, target);
		path.add(0, start);		

		/*System.out.println(path+"="+reObjs.dist.get(target));	*/

		//FORM a Path
		//		path.add(start);
		//		path.add(target);
		//
		//		double length=road_network.getDistanceBetween(start, target);
		//		System.out.println(path+"="+length);


		//Base case
		if (max_iter==0) {
			//System.out.println("<<<RG:"+start+","+target+","+budget+","+max_iter+","+path);
			return path;
		}

		//Benchmark influence
		double bmark_influence=social_network.getExpectedInfluence(union(path, iniActiveNodes));
		//Baseline influence
		double base_influence=social_network.getExpectedInfluence(iniActiveNodes);

		max_influence=bmark_influence-base_influence;

		//Enforce to be zero if both are the same.
		if(exclude(union(path, iniActiveNodes), start).equals(iniActiveNodes)) {
			max_influence=0;
		}

		ArrayList <V> path_1 = new ArrayList <V> ();
		ArrayList <V> path_2 = new ArrayList <V> ();

		//FILTER out reachable middle vertex set (this applies kind of forward-checking)
		HashSet<V> reachableMidVertexSet=new HashSet<V>();
		for (V mv: social_network.socialNetwork.getVertexList().keySet()){//middle node from social network RATHER routing network
			//for (V mv: road_network.getVertexList().keySet()){
			double vcTgt_1=targetVisitCost(mode, social_network.socialNetwork, mv, iniActiveNodes); //path: s-mv
			double vcTgt_2=targetVisitCost(mode, social_network.socialNetwork, target, iniActiveNodes);//path: mv-t
			double costFirstHalf=(reObjs.dist.get(mv)+vcTgt_1)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);
			double costSecondHalf=(dijkstraShortestPath(road_network, mv, target).length+vcTgt_2)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);
			double delta=0;
			if((costFirstHalf+costSecondHalf)<=(1-delta)*budget)
				reachableMidVertexSet.add(mv);			
		}

		//APPLY Best-subset algorithm (cost-discounted) to maintain a smaller set (size=k): best-k-mid-set
		HashSet<V> bestKmidSet=new HashSet<V>();
		int k=(mode==1)?Driver_dtd.midSetSize:Driver_rg.midSetSize;
		for(int i=0; i<k; i++){
			double ifdc_max=Double.NEGATIVE_INFINITY;
			V v_max = null;
			//find next candidate node
			if(reachableMidVertexSet.size()<=k) {
				bestKmidSet=reachableMidVertexSet;
				break;
			}

			for(V mv:reachableMidVertexSet){
				if (mv.equals(target)||mv.equals(start)||bestKmidSet.contains(mv)) continue;
				double vcTgt_1=targetVisitCost(mode, social_network.socialNetwork, mv, iniActiveNodes); //path: s-mv
				double vcTgt_2=targetVisitCost(mode, social_network.socialNetwork, target, iniActiveNodes);//path: mv-t
				double costFirstHalf=(reObjs.dist.get(mv)+vcTgt_1)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);
				double costSecondHalf=(dijkstraShortestPath(road_network, mv, target).length+vcTgt_2)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);
				HashSet <V> ns=new HashSet <V> ();
				ns.addAll(iniActiveNodes);
				ns.add(mv);
				double df=(social_network.getExpectedInfluence(union(path,ns))-base_influence)/(costFirstHalf+costSecondHalf);
				if((df>ifdc_max)){
					ifdc_max=df;
					v_max=mv;
					//System.out.println(df+","+v_max);
				}				
			}			
			//add to the set
			if(v_max!=null)
			bestKmidSet.add(v_max);
		}

		//for (V midVertex: road_network.getVertexList().keySet()){//FOR each middle point
		//for (V midVertex: reachableMidVertexSet){//FOR each reachable middle point
		for (V midVertex: bestKmidSet){//FOR each point in k-midset
			//System.out.println(midVertex);

			//This middle point can not be start, target and any point in initial set of active nodes
			if (midVertex.equals(target)||midVertex.equals(start)||iniActiveNodes.contains(midVertex)) continue; 

			boolean es=true; //use exponential split?
			//			for (double b=1; b<=budget; b++){//Linear Split
			//			double bFnl=b;
			for (double b=0; b<=Math.floor(Math.log(budget)/Math.log(2)); b++){//Exponential Split, (might increase search depth to lift utility)
				double bFnl=Math.pow(2, b);				

				//System.out.println("b="+b+",2^b="+bFnl);
				double distLB=reObjs.dist.get(midVertex)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);
				double vc=targetVisitCost(mode, social_network.socialNetwork, midVertex, iniActiveNodes)*((mode==1)?Driver_dtd.rescaleRatioRG:Driver_rg.rescaleRatio);

				if ((int)bFnl<(int)((distLB+vc))) {
					//b=Math.ceil(Math.log(distLB+vc)/Math.log(2))-1;//USE ceil to avoid Infinite Loop
					//b=distLB+vc-1;
					b=((es)?Math.ceil(Math.log(distLB+vc)/Math.log(2)):(distLB+vc))-1;
					continue; 
				}

				//Branch&bound
				path_1 = recusiveGreedy(mode, road_network, social_network, start, midVertex, (int) bFnl, iniActiveNodes, max_iter-1);
				/*System.out.println("Path_1:"+path_1);*/
				if (!path_1.isEmpty()) {				
					path_2 = recusiveGreedy(mode, road_network, social_network, midVertex, target, (int) (budget-bFnl), union(path_1,iniActiveNodes), max_iter-1);
					/*System.out.println("Path_2:"+path_2);*/

					if(!path_2.isEmpty()){
						path_2.remove(0);
						path_1.addAll(path_2);

						double new_influence=social_network.getExpectedInfluence(union(path_1, iniActiveNodes));					
						double delta_new=new_influence-base_influence;

						if(delta_new>max_influence)
						{
							path=path_1;
							max_influence=delta_new;
						}	
					} else break; //STOP process further for smaller budgets, since path_2 is already infeasible.
				}
			}		
		}		

		if(mode==1){
			if(max_iter==Driver_dtd.MAX_DEPTH_RG){
				//Driver_dtd.max_influe+=max_influence;
				Driver_dtd.max_influe=max_influence;
				Driver_dtd.budget=0;
				HashSet <V> v_set=new HashSet <V> ();

				//Visited node
				for(V v: path){
					if(!v.equals(start))
						v_set.add(v);
				}
				Driver_dtd.visit_set=exclude(v_set,iniActiveNodes);			

				//Budget
				for(int i=1; i<path.size(); i++){
					//System.out.println(dijkstraShortestPath(road_network, path.get(i-1), path.get(i)).length);
					V v=path.get(i);
					double visitCost=targetVisitCost(mode, social_network.socialNetwork, v, iniActiveNodes);
					Driver_dtd.budget+=(dijkstraShortestPath(road_network, path.get(i-1), path.get(i)).length+visitCost);
				}	
			}
		}else if(mode==2){
			if(max_iter==Driver_rg.MAX_DEPTH_RG){
				//Driver_rg.max_influe+=max_influence;
				Driver_rg.max_influe=max_influence;
				Driver_rg.budget=0;
				HashSet <V> v_set=new HashSet <V> ();

				//Visited node
				for(V v: path){
					if(!v.equals(start))
						v_set.add(v);
				}
				Driver_rg.visit_set=exclude(v_set,iniActiveNodes);			

				//Budget
				for(int i=1; i<path.size(); i++){
					//System.out.println(dijkstraShortestPath(road_network, path.get(i-1), path.get(i)).length);
					V v=path.get(i);
					double visitCost=targetVisitCost(mode, social_network.socialNetwork, v, iniActiveNodes);
					Driver_rg.budget+=(dijkstraShortestPath(road_network, path.get(i-1), path.get(i)).length+visitCost);
				}	
			}
		}
		//PAUSE for 1000 milliseconds=1sec.
		//		try {
		//			Thread.sleep(0);                 
		//		} catch(InterruptedException ex) {
		//			Thread.currentThread().interrupt();
		//		}

		//System.out.println("<<<RG:"+start+","+target+","+budget+","+max_iter+","+path);
		return path;		
	}

	public static <V> HashSet <V> union(ArrayList <V> path, HashSet <V> active_nodes){
		HashSet <V> union=new HashSet <V> (active_nodes);		
		for (V v: path){
			union.add(v);		
		}
		return union;		
	}

	public static <V> HashSet <V> union(HashSet <V> major, V node){
		HashSet <V> union=new HashSet <V> (major);		
		union.add(node);
		return union;		
	}

	public static <V> HashSet<V> exclude( HashSet <V> node_set_1, HashSet <V> node_set_2){
		HashSet <V> exclude=new HashSet <V> ();
		for(V v: node_set_1){
			if(!node_set_2.contains(v)) exclude.add(v);  
		}		
		return exclude;			
	}

	public static <V> HashSet<V> exclude( HashSet <V> node_set_1, V notThisNode){
		HashSet <V> exclude=new HashSet <V> (node_set_1);
		if(exclude.contains(notThisNode)) exclude.remove(notThisNode);		
		return exclude;			
	}

	public static <V> double targetVisitCost(int mode, Graph<V> network, V target, HashSet <V> iniActiveNodes){
		double vistCost=0;
		if(mode==1){
			vistCost=	(!network.getVertexList().containsKey(target)
					||target.equals(Driver_dtd.SOURCE_NODE)
					||target.equals(Driver_dtd.TARGET_NODE)
					||target.equals("S")//test case
					||iniActiveNodes.contains(target))?0:Driver_dtd.visit_cost;	
		}else if(mode==2){
			vistCost=	(!network.getVertexList().containsKey(target)
					||target.equals(Driver_rg.SOURCE_NODE)
					||target.equals(Driver_rg.TARGET_NODE)
					||target.equals("0")//test case
					||iniActiveNodes.contains(target))?0:Driver_rg.visit_cost;	

		}else if(mode==3){
			vistCost=	(!network.getVertexList().containsKey(target)
					//||target.equals(Driver_sp.SOURCE_NODE) start node 1023 can be visited, therefore it has a cost
					//||target.equals(Driver_sp.TARGET_NODE)
					//||target.equals("S")
					||iniActiveNodes.contains(target))?0:Driver_sp.visit_cost;
		}			
		return vistCost;	
	}
	public static <V> ArrayList<V> recusiveGreedy_sp(int mode, 
			Graph<V> sensorNet,
			DoubleMatrix cov, V start, V target, int budget,
			HashSet<V> iniActiveNodes, int max_iter) {
		//System.out.println(">>>RG:"+start+","+target+","+budget+","+max_iter);
		//System.out.println("Depth:"+max_iter);

		double maxCdep=0; //maximum conditional entropy
		ArrayList <V> path=new ArrayList <V> (); //Path thru which to seed

		//GET path by Dijkstra algorithm
		WrappedObject <V> reObjs=dijkstraShortestPath(sensorNet, start);

		//Return infeasible if out of budget
		double visitCostTarget=targetVisitCost(mode, sensorNet, target, iniActiveNodes);	
		double costLB=(reObjs.dist.get(target)+visitCostTarget)*Driver_sp.rescaleRatioRG;

		if((int)costLB>(int)budget) { //path cost s-t and visit cost of node t
			return path;
		}

		path.add(0, target);
		path.add(0, start);		

		//Base case
		if (max_iter==0) {
			return path;
		}

		//Base conditional entropy		
		double ceBase=MobileSensorUtil.getConditionalEntropy(cov, target, iniActiveNodes);
		maxCdep=ceBase;

		ArrayList <V> path_1 = new ArrayList <V> ();
		ArrayList <V> path_2 = new ArrayList <V> ();

		//FILTER out reachable middle vertex set (this applies kind of forward-checking)
		HashSet<V> reachableMidVertexSet=new HashSet<V>();
		for (V mv: sensorNet.getVertexList().keySet()){
			if (mv.equals(target)||mv.equals(start)) continue;
			double vcTgt_1=targetVisitCost(mode, sensorNet, mv, iniActiveNodes); //path: s-mv
			double vcTgt_2=targetVisitCost(mode, sensorNet, target, iniActiveNodes);//path: mv-t
			double costFirstHalf=(reObjs.dist.get(mv)+vcTgt_1)*Driver_sp.rescaleRatioRG;
			double costSecondHalf=(dijkstraShortestPath(sensorNet, mv, target).length+vcTgt_2)*Driver_sp.rescaleRatioRG;
			double delta=0;
			if((costFirstHalf+costSecondHalf)<=(1-delta)*budget) 
				reachableMidVertexSet.add(mv);			
		}

		//System.out.println("reachableMidVertexSet:"+reachableMidVertexSet);
		//APPLY Best-subset algorithm (cost-discounted) to maintain a smaller set (size=k): best-k-mid-set
		HashSet<V> bestKmidSet=new HashSet<V>();
		int k=Driver_sp.midSetSize;
		for(int i=0; i<k; i++){
			//System.out.println("k="+i);
			double ifdc_max=Double.NEGATIVE_INFINITY;
			V v_max = null;
			//find next candidate node
			if(reachableMidVertexSet.size()<=k) {
				bestKmidSet=reachableMidVertexSet;
				break;
			}
			for(V mv:reachableMidVertexSet){
				//System.out.println("mv:"+mv);
				//System.out.println("ini-set:"+iniActiveNodes);
				if (mv.equals(target)||mv.equals(start)||bestKmidSet.contains(mv)||iniActiveNodes.contains(mv)) continue;
				double vcTgt_1=targetVisitCost(mode, sensorNet, mv, iniActiveNodes); //path: s-mv
				double vcTgt_2=targetVisitCost(mode, sensorNet, target, iniActiveNodes);//path: mv-t
				double costFirstHalf=(reObjs.dist.get(mv)+vcTgt_1)*Driver_sp.rescaleRatioRG;
				double costSecondHalf=(dijkstraShortestPath(sensorNet, mv, target).length+vcTgt_2)*Driver_sp.rescaleRatioRG;

				//System.out.println("cost["+start+"->"+target+"]:"+costFirstHalf+costSecondHalf);
				//System.out.println("utility["+start+"->"+mv+"]:"+MobileSensorUtil.getConditionalEntropy(cov, mv, iniActiveNodes));//debug
				//System.out.println("utility["+mv+"->"+target+"]:"+MobileSensorUtil.getConditionalEntropy(cov, target, union(iniActiveNodes, mv)));

				double df=(MobileSensorUtil.getConditionalEntropy(cov, mv, iniActiveNodes)
						+MobileSensorUtil.getConditionalEntropy(cov, target, union(iniActiveNodes, mv)))
						/(costFirstHalf+costSecondHalf);
				//System.out.println("df:"+df+" ifdc_max:"+ifdc_max);


				if((df>ifdc_max)){
					ifdc_max=df;
					v_max=mv;
					//System.out.println(df+","+v_max);
				}				
			}			
			//add to the set
			//			if(v_max==null) {
			//				System.out.println("Catch it!:"+v_max);
			//			}
			if(v_max!=null)
				bestKmidSet.add(v_max);
			//System.out.println("BestKmidSet:"+bestKmidSet);
		}

		//System.out.println("BestKmidSet:"+bestKmidSet);

		//for (V midVertex: road_network.getVertexList().keySet()){//FOR each middle point
		//for (V midVertex: reachableMidVertexSet){//FOR each reachable middle point
		for (V midVertex: bestKmidSet){//FOR each point in k-midset
			//System.out.println(midVertex);

			//This middle point can not be start, target and any point in initial set of active nodes
			if (midVertex.equals(target)||midVertex.equals(start)||iniActiveNodes.contains(midVertex)) continue; 

			boolean es=true; //use exponential split?
			//			for (double b=1; b<=budget; b++){//Linear Split
			//			double bFnl=b;
			for (double b=0; b<=Math.floor(Math.log(budget)/Math.log(2)); b++){//Exponential Split, (might increase search depth to lift utility)
				double bFnl=Math.pow(2, b);				

				//System.out.println("b="+b+",2^b="+bFnl);				

				double distLB=reObjs.dist.get(midVertex)*Driver_sp.rescaleRatioRG;
				double vc=targetVisitCost(mode, sensorNet, midVertex, iniActiveNodes)*Driver_sp.rescaleRatioRG;

				if ((int)bFnl<(int)((distLB+vc))) {
					//b=Math.ceil(Math.log(distLB+vc)/Math.log(2))-1;//USE ceil to avoid Infinite Loop
					//b=distLB+vc-1;
					b=((es)?Math.ceil(Math.log(distLB+vc)/Math.log(2)):(distLB+vc))-1;
					continue; 
				}

				//Branch&bound

				path_1 = recusiveGreedy_sp(mode, sensorNet, cov, start, midVertex, (int) bFnl, iniActiveNodes, max_iter-1);
				/*System.out.println("Path_1:"+path_1);*/
				if (!path_1.isEmpty()) {				
					path_2 = recusiveGreedy_sp(mode, sensorNet, cov, midVertex, target, (int) (budget-bFnl), union(path_1,iniActiveNodes), max_iter-1);
					/*System.out.println("Path_2:"+path_2);*/

					if(!path_2.isEmpty()){
						path_2.remove(0);
						path_1.addAll(path_2);

						//Use chain rules to compute conditional entropy of current path/visiting nodes given initial active nodes.
						//H(P_1P_2X)=H(P_2|P_1X)+H(P_1|X)
						double delta_new=0;						
						HashSet <V> X=new HashSet <V> (iniActiveNodes);

						for(int i=1; i<path_1.size(); i++){
							V v=path_1.get(i);
							delta_new+=MobileSensorUtil.getConditionalEntropy(cov, v, X);
							X.add(v);
						}

						if(delta_new>maxCdep)
						{
							path=path_1;
							maxCdep=delta_new;
						}	
					} else break; //STOP process further for smaller budgets, since path_2 is already infeasible.
				}
			}		
		}		

		if(mode==3){
			if(max_iter==Driver_sp.MAX_DEPTH_RG){
				Driver_sp.max_entropy=maxCdep;
				Driver_sp.budget=0;
				HashSet <V> v_set=new HashSet <V>  ();

				//Visited node
				for(V v: path){
					//if(!v.equals(start)) //1023 can be visited
						v_set.add(v);
				}
				Driver_sp.visit_set=exclude(v_set,iniActiveNodes);			

				//Budget
				for(int i=1; i<path.size(); i++){
					//System.out.println(dijkstraShortestPath(road_network, path.get(i-1), path.get(i)).length);
					V v=path.get(i);
					double visitCost=targetVisitCost(mode, sensorNet, v, iniActiveNodes);
					Driver_sp.budget+=(dijkstraShortestPath(sensorNet, path.get(i-1), path.get(i)).length+visitCost);
				}	
			}
		}
		//System.out.println("<<<RG:"+start+","+target+","+budget+","+max_iter+","+path);
		return path;	
	}


}

