package submodularMaxOverGraph;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.Stack;

import org.jblas.*;

import static org.jblas.DoubleMatrix.*;
import static org.jblas.MatrixFunctions.*;

/**
 * Driver program for optimal mobile sensor placement
 * @author zhangh24
 *
 */
public class Driver_sp {	
	public static int max_budget; 
	public static double visit_cost;
	public static int max_iteration;
	public static double max_entropy; //submodular objective:entropy
	public static double budget; //budget used
	public static HashSet visit_set; //visit set
	public final static int MODE=3; //Experiment Mode: 1:door-to-door, 2:random-graph, 3:robot-sensor
	public static boolean prune_mode; //prone mode
	public static  int MAX_DEPTH_RG; //Maximum depth of recursive greedy
	public static final int rescaleRatioRG=(int) Math.pow(10, 0);//Re-scale cost and budget
	public final static int MAX_RUN=1;
	public static final String SOURCE_NODE = "1022";//H:1022, M:1023, L:1033
	public static final String TARGET_NODE = "1022";
	private static int MAX_AGENT = 3;
	public static int midSetSize;
	public static int maxIterISK;
	public static boolean OPT_K=true;//indicate optimal k experiment
	public static boolean RANDOM_START=true;//indicate random start/target


	public static void main(String[] args) {
		//1st argument specify algorithm
		String algo=args[0];
		//2nd argument specify prune mode
		if(args[1].equals("T")) prune_mode=true;
		else prune_mode=false;

		if(algo.equals("rg")){
			midSetSize=Integer.valueOf(args[2]);
			MAX_DEPTH_RG=Integer.valueOf(args[3]);
			//MAX_DEPTH_RG=3;//manually assign search depth
		}

		if(algo.equals("isk")){
			maxIterISK=Integer.valueOf(args[2]);			
		}

		//[AIJ optimal k]Last argument: number of agents
		if(OPT_K)
			MAX_AGENT=Integer.valueOf(args[args.length-1]);

		Graph<String> sensorNet = new Graph<String>(false);

		//Covariance Matrix and inverse, i.e., 36*36
		DoubleMatrix cov=new DoubleMatrix(36,36);
		DoubleMatrix cov_ivs=new DoubleMatrix(36,36);

		//LOAD sensor network: used to compute cost
		DataLoader.loadNetwork(sensorNet, false, "senet_edges.csv", "senet_nodes.csv");		

		//LOAD covariance matrix: used to compute entropy
		DataLoader.loadMatrix(cov,"bg_temp_cv_36.csv");		
		//DataLoader.loadMatrix(cov_ivs, "bg_temp_cv_ivs_36.csv");			

		//Test Network
		String source = SOURCE_NODE;
		String target = TARGET_NODE;

		//System.out.println("Start here ... ");
		//System.out.println(sensorNet.toString());

		HashSet <String> ini_set=new HashSet <String> ();
		//ini_set.add("1023");

		System.out.println("Application: mobile sensoring");
		System.out.println("Algorithm:"+algo);
		if(algo.equals("rg")){
			System.out.println("Middle set size:"+midSetSize);
			System.out.println("Max Depth:"+MAX_DEPTH_RG);
		}else if(algo.equals("isk")){
			System.out.println("Max Iteration:"+maxIterISK);
		}
		System.out.println("Prune Mode:"+prune_mode);



		visit_set=new HashSet();

		//SINGLE AGENT		
		//		for (double c=0; c<=200; c+=10){
		//			visit_cost=c/10;
		//			for (int b=0; b<=200; b+=20){
		//				max_budget=b;
		//				for (int m=1; m<=1; m++){
		//					max_iteration=m;


		//		System.out.println("max_bgt;visit_cost;max_iter;walk;visit;bgt;max_entropy;time");
		//		for (double c=0; c<=200; c+=10){
		//			visit_cost=c/10;
		//			for (int b=0; b<=0; b+=20){
		//				max_budget=200;
		//				for (int m=1; m<=1; m++){
		//					max_iteration=m;
		//					System.out.print(max_budget+";"+visit_cost+";"+max_iteration+";");
		//					max_entropy=0;
		//					long start = System.nanoTime();
		//					ArrayList <String> walk=new ArrayList();
		//
		//					for (int i=0; i<MAX_RUN;i++){	
		//
		//						//RG
		//						if(algo.equals("rg")){							
		//							//Driver.visit_set=new HashSet();
		//							walk=Algorithm3.recusiveGreedy_sp(MODE, sensorNet, cov, source, target, max_budget*rescaleRatioRG, ini_set, MAX_DEPTH_RG);
		//						}						
		//
		//						//GCB
		//						if(algo.equals("gcb"))
		//							walk=Algorithm4.greedyWalk_sp(sensorNet, cov, source, target, max_budget, ini_set);
		//
		//						//GR
		//						if(algo.equals("gr"))
		//							walk=Algorithm5.greedyWalk_sp(sensorNet, cov, source, target, max_budget, ini_set);
		//
		//						//ISK
		//						if(algo.equals("isk"))
		//							walk=Algorithm6.greedyWalk_sp(MODE, sensorNet, cov, source, target, max_budget, ini_set);
		//
		//						System.out.print(walk+";");
		//						System.out.print(visit_set+";");
		//						System.out.print(budget+";");
		//
		//					}
		//					System.out.print(max_entropy/MAX_RUN+";");
		//					System.out.println((System.nanoTime() - start)/1.0e+9/MAX_RUN);//Average time per run
		//				}
		//			}
		//		}

		//N-AGENT[sequential allocation]
		//		System.out.println("Number of agents:"+MAX_AGENT);		
		System.out.println("agent;max_bgt;visit_cost;max_iter;walk;visit;bgt;max_influ;time");
		//AIJ initial configuration
		//		for (double c=0; c<=200; c+=20){//10
		//			visit_cost=c/10;
		//			for (int b=0; b<=200; b+=20){//10
		//				max_budget=b;

		//[AIJ optimal k] configuration
		for (double c=0; c<=100; c+=100){//0, 10
			visit_cost=c/10;
			for (int b=100; b<=200; b+=50){//100 to 500->100, 150, 200
				max_budget=b/MAX_AGENT;

				for (int m=1; m<=1; m++){
					max_iteration=m;
					HashSet <String> iniSet_m=new HashSet <String> (ini_set);//Map a copy of iniSet

					Random myRandom=new Random(2016);
					double remainBgt=0;//remaining budget by last actor
                    double locationPickingCost=0;//cost to pick up a location

					for (int i=0; i<MAX_AGENT;i++){ //for each agent
						System.out.print(i+";"+max_budget+";"+visit_cost+";"+max_iteration+";");
						long start = System.nanoTime();
						//reset budget, max influence and walk
						budget=0;						
						max_entropy=0;
						ArrayList <String> walk=new ArrayList <String> ();

						//[AIJ optimal k] Random Assign starting and ending location
						Set <String> vertexNames=sensorNet.getVertexList().keySet();
						int r=myRandom.nextInt(vertexNames.size());
						int idx=0;
						for(String s: vertexNames){
							if(idx==r){
								if(RANDOM_START){
									source=s;
									target=s;
								}
							}
							idx++;
						}	
						//[AIJ optimal k] Assign location picking cost, 10
						locationPickingCost=(RANDOM_START)?10:0;

						//RG
						if(algo.equals("rg"))							
							walk=Algorithm3.recusiveGreedy_sp(MODE, sensorNet, cov, source, target, max_budget*rescaleRatioRG, iniSet_m, MAX_DEPTH_RG);						
						//GCB
						if(algo.equals("gcb"))
							walk=Algorithm4.greedyWalk_sp(sensorNet, cov, source, target, max_budget+remainBgt-locationPickingCost, iniSet_m);
						//GR
						if(algo.equals("gr"))
							walk=Algorithm5.greedyWalk_sp(sensorNet, cov, source, target, max_budget, iniSet_m);
						//ISK
						if(algo.equals("isk"))
							walk=Algorithm6.greedyWalk_sp(MODE, sensorNet, cov, source, target, max_budget, iniSet_m);
						long end = System.nanoTime();						
						System.out.println(walk+";"+visit_set+";"+(budget+locationPickingCost)+";"+max_entropy+";"+(end - start)/1.0e+9);			
						//update initial active set
						iniSet_m.addAll(visit_set);
						//reset visit set
						visit_set.clear();
						
						//[AIJ optimal k]collect unused budget
						remainBgt=max_budget+remainBgt-budget-locationPickingCost;
						//System.out.println(remainBgt);
					}					
				}
			}
		}
	}
}


