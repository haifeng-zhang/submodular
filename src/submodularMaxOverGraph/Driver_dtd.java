package submodularMaxOverGraph;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.Stack;
/**
 * Driver program for optimal routing over influence networks (door-to-door marketing).
 * @author zhangh24
 *
 */
public class Driver_dtd {
	public static double max_budget;
	public static double visit_cost;
	public static int max_iteration;
	public static double max_influe;
	public static double budget; //budget used
	public static HashSet visit_set; //visit set
	public static boolean prune_mode; //prone mode: feasible node only for GCB, GR, and ISK
	public final static int MODE=1; //Experiment Mode: 1:door-to-door, 2:random-graph
	public static int rescaleRatioRG=1;//Re-scale cost and budget
	public static int MAX_DEPTH_RG;//Maximum depth of recursive greedy
	public final static int MAX_RUN=1;
	public static final String SOURCE_NODE="49133440";//start node
	public static final String TARGET_NODE="49133440";//ending node
	private static int MAX_AGENT = 3;
	public static int midSetSize;//mid set size of recursive greedy
	public static int maxIterISK;
	public static boolean OPT_K=true;//indicate optimal k experiment
	public static boolean RANDOM_START=false;//indicate random start/target

	public static void main(String[] args) {
		//1st argument specify algorithm
		String algo=args[0];//i.e., cb(cost benefit), gr(greedy), isk(iterative submodular knapsack), rg(recursive greedy)
		//2nd argument specify prune mode (pruning mode improves utilities)
		if(args[1].equals("T")) prune_mode=true;
		else prune_mode=false;

		if(algo.equals("rg")) {
			midSetSize=Integer.valueOf(args[2]);
			MAX_DEPTH_RG=Integer.valueOf(args[3]);
		}

		if(algo.equals("isk")){
			maxIterISK=Integer.valueOf(args[2]);			
		}


		//[AIJ optimal k]Last argument: number of agents
		if(OPT_K)
			MAX_AGENT=Integer.valueOf(args[args.length-1]);

		//double frm=Double.parseDouble(args[2]);
		Graph<String> roadNet = new Graph<String>(false);
		Graph<String> sociNet = new Graph<String>(true);

		//USE test case (i.e., a simple graph with 7 nodes) or not
		boolean test_case=false;
		if(!test_case) rescaleRatioRG=(int) Math.pow(10, 3);

		//BUILD an undirected graph using edge-list and node-list files
		//BUILD a routing network
		if (test_case) DataLoader.loadNetwork(roadNet, false, "edges_road_test.csv", "nodes_road_test.csv");
		else DataLoader.loadNetwork(roadNet, false, "edges_road.csv", "nodes_road.csv");

		//BUILD a social network 
		if (test_case) DataLoader.loadNetwork(sociNet, true, "edges_soci_test.csv", "nodes_soci_test.csv");
		else DataLoader.loadNetwork(sociNet, true, "edges_soci.csv", "nodes_soci.csv");
		//ATTACH to an Independent cascade (IC) model
		InfluenceModel <String> soci_model=new InfluenceModel <String> (sociNet);

		//Source and Target of the path
		String source = "";
		String target = "";

		if (test_case) {
			source = "S";
			target = "S";
		} else {
			source = SOURCE_NODE;
			target = TARGET_NODE;
		}

		//System.out.println("Start here ... ");
		//System.out.println(roadNet.toString());
		//System.out.println(sociNet.toString());

		//Initial active node set
		HashSet <String> iniSet=new HashSet <String> ();
		if (test_case) iniSet.add("A");
		else iniSet.add("188397");

		//COMPUTE gamma, beta, k_c
		//		System.out.println("visit_cost, gamma, beta, k_c");
		//		for(double c=0.0; c<=3;c+=0.1){
		//			System.out.print(c+",");
		//		    Algorithm7.estimateCurvature(roadNet, soci_model, source, c);
		//		}

		//ESTIMTE alpha
		//		System.out.println("visit_cost, alpha");
		//		for(double c=0.0; c<=3;c+=0.1){
		//			System.out.print(c+",");
		//			Algorithm7.estimateAlpha(roadNet, soci_model, source, 0, 100);	
		//		}




		//test Christofides algorithm
		//		HashSet <String> cover_nodes = new HashSet <String> ();
		//		String [] nodes={"A", "B", "C"};
		//		for(String s: nodes){
		//			cover_nodes.add(s);
		//		}
		//		
		//		if(false) {
		//			Graph.shortestCoverCostChristofides(source, roadNet, cover_nodes);			
		//			return;
		//		}		
		//		

		//Verfication
		//		String [] path={"49133440", "53760", "75281", "185195", "49133440"};
		//		double length=0;
		//		for(int i=1; i<path.length;i++){
		//			length+=Graph.dijkstraShortestPath(roadNet, path[i], path[i-1]).length;
		//		}
		//		System.out.println(length);

		//[AIJ optimal k] Extract Way points
		LinkedHashSet <String> waypoints=new LinkedHashSet <String> ();
		for(String s:roadNet.getVertexList().keySet()){
			if(!sociNet.getVertexList().keySet().contains(s))
				waypoints.add(s);
		}		
		
		//LOGING header
		System.out.println("Application: door-to-door marketing");
		System.out.println("Algorithm:"+algo);
		if(algo.equals("rg")){
			System.out.println("Middle set size:"+midSetSize);
			System.out.println("Max Depth:"+MAX_DEPTH_RG);
		}else if(algo.equals("isk")){
			System.out.println("Max Iteration:"+maxIterISK);
		}
		System.out.println("Use prune mode:"+prune_mode);

		visit_set=new HashSet();	

		//Table of Parameters
		//Experiment		Budget		Visit Cost			Rescaled
		//test_graph		30			[0, 5, by=1]		1
		//dtd_vc			3			[0,1,by=0.1]		1*10^3
		//dtd_vb			[0,1,by=0.2][0,0.2, by=0.1]	    1*10^3		

		System.out.println("max_bgt;visit_cost;max_iter;walk;visit;bgt;max_influ;time");
		//		//SINGLE AGENT
		//		//Main Loop
		//		for (double vc=0; vc<=0; vc+=1){//visit cost
		//			//visit_cost=vc/10;
		//			visit_cost=(test_case)?0:0;
		//			for (double b=30; b<=30; b+=10){//budget
		//				//max_budget=b/10;				
		//				max_budget=(test_case)?30:3; 
		//				for (int m=1; m<=10; m++){//iteration
		//					max_iteration=m;
		//					System.out.print(max_budget+";"+visit_cost+";"+max_iteration+";");
		//					max_influe=0;
		//					long start = System.nanoTime();
		//					ArrayList <String> walk=new ArrayList();
		//					for (int i=0; i<MAX_RUN;i++){	
		//
		//						//RG
		//						if(algo.equals("rg")){							
		//							//visit_set=new HashSet();
		//							walk=Algorithm3.recusiveGreedy(MODE, roadNet, soci_model, source, target, (int) max_budget*rescaleRatioRG, iniSet, MAX_DEPTH_RG);
		//						}
		//
		//						//GCB
		//						if(algo.equals("gcb"))
		//							walk=Algorithm4.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget, iniSet);
		//
		//						//GR
		//						if(algo.equals("gr"))
		//							walk=Algorithm5.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget, iniSet);
		//
		//						//ISK
		//						if(algo.equals("isk"))
		//							walk=Algorithm6.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget, iniSet);
		//
		//						System.out.print(walk+";");
		//						System.out.print(visit_set+";");
		//						System.out.print(budget+";");
		//
		//					}
		//					System.out.print(max_influe/MAX_RUN+";");
		//					System.out.println((System.nanoTime() - start)/1.0e+9/MAX_RUN); //Average time per run
		//				}
		//			}
		//		}


		//N-AGENT[sequential allocation]
		System.out.println("Number of agents:"+MAX_AGENT);
		System.out.println("agent;max_bgt;visit_cost;max_iter;walk;visit;bgt;max_influ;time");
		//AIJ initial configuration
		//		for (double c=0; c<=10; c+=1){
		//			visit_cost=c/10.0;
		//			for (double b=35; b<=60; b+=5){  //0:3, 3:6
		//				//max_budget=(test_case)?20:3;
		//				max_budget=b/10;

		//[AIJ optimal k] configuration
		for (double c=0; c<=1; c+=1){ //0, 0.1
			visit_cost=c/10.0;
			for (double b=10; b<=30; b+=10){  //1 to 3
				//max_budget=(test_case)?20:3;
				max_budget=b/(10*MAX_AGENT);

				for (int m=1; m<=1; m++){
					max_iteration=m;
					HashSet <String> iniSet_m=new HashSet <String> (iniSet);//Map a copy of iniSet

					Random myRandom=new Random(2016+m-1);  //each iteration use a different seed
					double remainBgt=0;//remaining budget by last actor
                    double locationPickingCost=0;//cost to pick up a location

					for (int i=0; i<MAX_AGENT;i++){ //for each agent						
						System.out.print(i+";"+max_budget+";"+visit_cost+";"+max_iteration+";");
						long start = System.nanoTime();

						//reset budget, max influence and walk
						budget=0;						
						max_influe=0;
						ArrayList <String> walk=new ArrayList <String> ();

						//[AIJ optimal k] Random Assign starting and ending location
						//Set <String> vertexNames=roadNet.getVertexList().keySet();
						
						int r=myRandom.nextInt(waypoints.size());
						int idx=0;
						for(String s: waypoints){
							if(idx==r){
								if(RANDOM_START){
									source=s;
									target=s;
								}
							}
							idx++;
						}						
						
						//[AIJ optimal k] Assign location picking cost, 0, 0.1, 0.2
						//locationPickingCost=(RANDOM_START)?0.2:0;
						locationPickingCost=(RANDOM_START)?0.2:0;  //pc1-5:[0, 0.05, 0.1, 0.15, 0.2]

						//RG
						if(algo.equals("rg")){							
							walk=Algorithm3.recusiveGreedy(MODE, roadNet, soci_model, source, target, (int) max_budget*rescaleRatioRG, iniSet_m, MAX_DEPTH_RG);
						}						

						//GCB
						if(algo.equals("gcb"))
							walk=Algorithm4.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget+remainBgt-locationPickingCost, iniSet_m);

						//GR
						if(algo.equals("gr"))
							walk=Algorithm5.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget, iniSet_m);

						//ISK
						if(algo.equals("isk"))
							walk=Algorithm6.greedyWalk(MODE, roadNet, soci_model, source, target, max_budget, iniSet_m);

						long end = System.nanoTime();
						System.out.println(walk+";"+visit_set+";"+(budget+locationPickingCost)+";"+max_influe+";"+(end - start)/1.0e+9);			
						//update initial active set
						iniSet_m.addAll(visit_set);
						//reset visit set
						visit_set.clear();
						
						
						//[AIJ optimal k]collect unused budget, NOT USE THIS
						//remainBgt=max_budget+remainBgt-budget-locationPickingCost;
						//System.out.println(remainBgt);
					}					
				}
			}
		}

		/*
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
		 */
		/*
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

		 */
	}
}
