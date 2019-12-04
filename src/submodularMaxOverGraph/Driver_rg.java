package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Random;
import java.util.Set;
/**
 * Driver program for sub-modular maximization over random graph.
 * @author zhangh24
 *
 */
public class Driver_rg {
	public static double max_budget;
	public static double visit_cost;
	public static int max_iteration;
	public static double max_influe;
	public static double budget; //budget used
	public static boolean prune_mode; //prone mode
	public static HashSet visit_set; //visit set
	public final static int MODE=2; //Experiment Mode: 1:door-to-door, 2:random-graph
	public final static int MAX_RUN=1;//Maximum number of runs	
	public static final int rescaleRatio=(int) Math.pow(10,2);//Re-scale ratio (of cost) for recursive greedy
	public static int MAX_DEPTH_RG;//Maximum depth for recursive greedy
	public static String SOURCE_NODE ; //Source node
	public static String TARGET_NODE ; //Target node
	public static int midSetSize;
	private static int MAX_AGENT = 3;
	public static String graphSize=null;
	public static String rtnet;
	public static int maxIterISK;
	public static boolean OPT_K=true;//indicate optimal k experiment
	public static boolean RANDOM_START=true;//indicate random start/target

	public static void main(String[] args) {
		//1st argument specify algorithm
		//i.e., gcb(generalized cost benefit), gr(greedy), isk(iterative submodular knapsack)
		String algo=args[0];
		//2nd argument specify prune mode
		if(args[1].equals("T")) prune_mode=true;
		else prune_mode=false;				

		Graph<String> roadNet = new Graph<String>(false);
		Graph<String> sociNet = new Graph<String>(false);

		//USE test graph?
		boolean test_case=false;
		if(test_case){
			SOURCE_NODE="0";
			TARGET_NODE="0";			
			//3th argument chooses size x*10 of test graphs			
			graphSize=args[2]; //[1,2,3,4,5]
			System.out.println("Random graph with size:"+graphSize+"0");
			//4rd argument specify route network [1:11]=[0.1,0.2, by=0.01][8*,9,10]
			rtnet=args[3];
			if(algo.equals("rg")){
				midSetSize=Integer.valueOf(args[4]);
				MAX_DEPTH_RG=Integer.valueOf(args[5]);
			}else if(algo.equals("isk")){
				maxIterISK=Integer.valueOf(args[4]);			
			}

		}else{
			SOURCE_NODE="2";
			TARGET_NODE="2";
			//3rd argument specify route network [1:10]=[0.01,0.1, by=0.01]
			//AAAI16: 1:3
			rtnet=args[2];
			if(algo.equals("rg")){//skip rg for non-test graph
				midSetSize=Integer.valueOf(args[3]);
				MAX_DEPTH_RG=Integer.valueOf(args[4]);	
			}else if(algo.equals("isk")){
				maxIterISK=Integer.valueOf(args[3]);			
			}
		}

		//[AIJ optimal k]Last argument: number of agents
		if(OPT_K)
			//MAX_AGENT=Integer.valueOf(args[args.length-1]);
			MAX_AGENT=Integer.valueOf(args[3]);

		//BUILD undirected graph by edge-list and node-list files
		String rtnet_nodes = null;
		String rtnet_edges = null;
		if(test_case){
			rtnet_nodes="data/RandomGraph/test/n"+graphSize+"0/rg_er_nodes_"+rtnet+".csv";
			rtnet_edges="data/RandomGraph/test/n"+graphSize+"0/rg_er_rtNet_"+rtnet+".csv";
		}else{
			rtnet_nodes="data/RandomGraph/rg_er_nodes_"+rtnet+".csv";
			rtnet_edges="data/RandomGraph/rg_er_rtNet_"+rtnet+".csv";
		}
		DataLoader.loadNetwork(roadNet, false, rtnet_edges, rtnet_nodes);

		//BUILD an social network, i.e., independent cascade model 
		String ifnet_nodes = null;
		String ifnet_edges = null;
		if(test_case){
			ifnet_nodes="data/RandomGraph/test/n"+graphSize+"0/rg_ba_nodes_2.csv";
			ifnet_edges="data/RandomGraph/test/n"+graphSize+"0/rg_ba_ifNet_2.csv";
		}else{
			ifnet_nodes="data/RandomGraph/rg_ba_nodes_2.csv";
			ifnet_edges="data/RandomGraph/rg_ba_ifNet_2.csv";
		}

		DataLoader.loadNetwork(sociNet, false, ifnet_edges, ifnet_nodes);//note undirected social graph

		//Test Network
		String source = SOURCE_NODE;
		String target = TARGET_NODE;

		//System.out.println("Start here ... ");
		//System.out.println(roadNet.toString());
		//System.out.println(sociNet.toString()); 

		//Initial active set
		HashSet <String> iniSet=new HashSet <String> ();
		iniSet.add("1");

		InfluenceModel <String> soci_model=new InfluenceModel <String> (sociNet);
		//double inf=0;

		//for(int i=0; i<1000; i++){
		//inf+=my_model.getExpectedInfluence(s);		    
		//}

		//System.out.println(inf/1000);

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

		//		HashSet <String> coverSet=new HashSet <> ();
		//		coverSet.add("1");
		//		coverSet.add("2");
		//		coverSet.add("3");
		//		coverSet.add("4");
		//		coverSet.add("5");
		//
		//
		//		long end0, end1;
		//		ArrayList <String> walk_test;
		//		for(int i=0; i<1000; i++){
		//			//TEST TWO COST COMPUATION
		//			end0 = System.nanoTime();
		//			//GREEDY
		//			ArrayList <String> walk=Graph.shortestCoverCostGreedy(source,roadNet,coverSet).walk;
		//			//walk_test=Graph.shortestCoverCostChristofides(source,roadNet,coverSet).walk;
		//			end1=System.nanoTime();
		//			//CHRISTOFIDE
		//			//Graph.shortestCoverCostChristofides(source,roadNet,coverSet);
		//			//long end2=System.nanoTime();
		//			//System.out.println("gr,ch:"+(end1-end0)/1e+6+","+(end2-end1)/1e+6);
		//			System.out.println((end1-end0)/1e+6);
		//		}


		//		for(int i=0; i<100; i++){
		//			long end2=System.nanoTime();
		//			//CHRISTOFIDE
		//			Graph.shortestCoverCostChristofides(source,roadNet,coverSet);
		//			long end3=System.nanoTime();
		//			//GREEDY
		//			Graph.shortestCoverCostGreedy(source,roadNet,coverSet);
		//			long end4=System.nanoTime();
		//			System.out.println("ch,gr:"+(end3-end2)/1e+6+","+(end4-end3)/1e+6);
		//		}



		System.out.println("Application: door-to-door marketing with random graphs");
		System.out.println("Algorithm:"+algo);
		if(algo.equals("rg")){
			System.out.println("Middle set size:"+midSetSize);
			System.out.println("Max Depth:"+MAX_DEPTH_RG);
		}else if(algo.equals("isk")){
			System.out.println("Max Iteration:"+maxIterISK);
		}
		System.out.println("Prune Mode:"+prune_mode);
		System.out.println("ER graph:"+rtnet);		

		visit_set=new HashSet();

		//aaai16 configuration is as follows:
		//		for (double c=0; c<=10; c+=5){
		//			visit_cost=c/10;
		//			for (double b=0; b<=10; b+=1){
		//				max_budget=b;				
		//				for (int m=1; m<=10; m++){
		//					max_iteration=m;

		//SINGLE AGENT
		//		System.out.println("max_bgt;visit_cost;max_iter;walk;visit;bgt;max_influ;time");
		//		for (double c=0; c<=0; c+=5){
		//			visit_cost=1;
		//			for (double b=0; b<=0; b+=1){
		//				max_budget=10;
		//				for (int m=1; m<=10; m++){
		//					max_iteration=m;
		//					System.out.print(max_budget+";"+visit_cost+";"+max_iteration+";");
		//					max_influe=0;
		//					long start = System.nanoTime();
		//					ArrayList <String> walk=new ArrayList <String> ();
		//					for (int i=0; i<MAX_RUN;i++){	
		//
		//						//RG
		//						if(algo.equals("rg")){							
		//							walk=Algorithm3.recusiveGreedy(MODE, roadNet, soci_model, source, target, (int) max_budget*rescaleRatio, iniSet, MAX_DEPTH_RG);
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
		//					}
		//					System.out.print(max_influe/MAX_RUN+";");
		//					System.out.println((System.nanoTime() - start)/1.0e+9/MAX_RUN); //Average time per run
		//				}
		//
		//			}
		//		}

		//N-AGENT[sequential allocation]
		System.out.println("agent;max_bgt;visit_cost;max_iter;walk;visit;bgt;max_influ;time");

		//AIJ initial configuration
		//Configuration for test case
		//		for (double c=0; c<=10; c+=2){
		//			visit_cost=c/10.0;
		//			for (double b=5; b<=10; b+=5){
		//				max_budget=b;
		//Configuration for regular case
		//		for (double c=0; c<=10; c+=1){
		//			visit_cost=c/10.0;
		//			for (double b=12; b<=20; b+=2){   //0 to 10, and 12 to 20
		//				max_budget=b;

		double [] pcs={0, 0.1, 0.2, 0.3, 0.4, 0.5, 1, 1.5, 2, 2.5, 3}; 
		
		//[AIJ optimal k] configuration
		for (double c=5; c<=5; c+=5){ //0, 0.5
			visit_cost=c/10.0;
			for (double b=10; b<=30; b+=10){   //10 to 30
				max_budget=b/MAX_AGENT;
				
				for (int m=1; m<=10; m++){
					max_iteration=m;
					HashSet <String> iniSet_m=new HashSet <String> (iniSet);//Map a copy of iniSet

					Random myRandom=new Random(2016+m-1);
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
						LinkedHashMap <String, Vertex> vertexNames= roadNet.getVertexList();
						int r=myRandom.nextInt(vertexNames.size());
						int idx=0;
						for(String s: vertexNames.keySet()){
							if(idx==r){
								if(RANDOM_START){
									source=s;
									target=s;
								}
							}
							idx++;
						}
						//[AIJ optimal k] Assign location picking cost, 10
						//locationPickingCost=(RANDOM_START)?1:0;
						locationPickingCost=pcs[Integer.valueOf(args[4])];  //pc0-5: [0, 0.1, 0.2, 0.3, 0.4, 0.5]
						
						//RG
						if(algo.equals("rg")){							
							walk=Algorithm3.recusiveGreedy(MODE, roadNet, soci_model, source, target, (int) max_budget*rescaleRatio, iniSet_m, MAX_DEPTH_RG);
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

						long end =System.nanoTime();
						System.out.println(walk+";"+visit_set+";"+(budget+locationPickingCost)+";"+max_influe+";"+(end - start)/1.0e+9);			
						//update initial active set
						iniSet_m.addAll(visit_set);
						//reset visit set
						visit_set.clear();
						
						//[AIJ optimal k]collect unused budget
						//remainBgt=max_budget+remainBgt-budget-locationPickingCost;
						//System.out.println(remainBgt);
					}					
				}
			}
		}
	}
	//}
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
