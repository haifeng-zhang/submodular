package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

public class InfluenceModel<V> {

	Graph<V> socialNetwork; //Underlying social network structure
	private final int MAX_STAGES=12; //max number of stages, higher and larger variance of expected influence
	private final int MAX_RUNS=5000; //max number of sample model runs, higher and smaller variance of expected influence: 
	//NOTE: 10 is never enough/robust
	//AAAI2016:100
	//jair: 1000 [compare]
	//jair: [opt k]dtd:10000, rg:5000


	public InfluenceModel(Graph <V> graph) {
		socialNetwork=graph;
	}

	/**
	 * Compute influence of a single model run
	 * @param active_nodes set of active nodes
	 * @return influence
	 */
		public int getInfluence_v1(HashSet <V> active_nodes){
			int influence=active_nodes.size();
	
			//System.out.println("~~~~~~~~Stage:0~~~~~~~~~");
			//System.out.println("Seeds:"+active_nodes.toString());
	
			//ASSIGN states of all vertices according the list of active nodes
			for (Vertex vertex : socialNetwork.getVertexList().values()) {			
				if(active_nodes.contains(vertex.getID())) vertex.activate();
				else vertex.reset();
			}		
	
			//IMPLEMENT the independent cascade process
			for (int t=0; t<this.MAX_STAGES; t++){	
				
				//System.out.println("ITR:"+t);

	
				//UPDATE states stage by stage discretely
				for (Vertex vertex : socialNetwork.getVertexList().values()) {			
					vertex.updateState();
					//System.out.println(vertex.getID()+":"+vertex.is_activated_curr()+","+vertex.is_activated_next()+","+vertex.is_capable());

				}	
	
				//System.out.println("~~~~~~~~Stage:"+(t+1)+"~~~~~~~~~");
	
				for (Vertex vertex : socialNetwork.getVertexList().values()) {
	
					//System.out.println("Current Vertex:"+vertex.getID());
	
					if(!vertex.is_activated_curr()) { // if not been activated
	
						ArrayList <V> nb_IDs= socialNetwork.getAdjacentVertices((V) vertex.getID());
						//System.out.println(nb_IDs.toString());
	
						for (V v_id: nb_IDs){ //for each adjacent vertex
							//System.out.println(">>Neighbor:"+v_id);
	
							Vertex v=socialNetwork.getVertexList().get(v_id); //get actual node object thru its id
							//System.out.println(v.is_activated_curr()+","+v.is_activated_next()+","+v.is_capable());

	
							double prob;// = socialNetwork.getDistanceBetween(v_id, (V) vertex.getID()); //get edge weight, i.e. activation probability
							//NEED A FAST REPRENTATION to get weight...
							//TEST ONLY prob=0.5, 0.8
							prob=0.1;
	
							//System.out.println("Activation Prob:"+prob);
	
							if (v.is_capable()) { // if its neighbor is active
	
								//Get activated at a chance determined by the weight
								Random rn = new Random(System.nanoTime());
								double rn_db=rn.nextDouble();
								//System.out.println("Random Draw:"+rn_db);
	
								if (rn_db<=prob) {
									//System.out.println("ACTIVATED!!!");
									if(!vertex.is_activated_next()) influence++; 
									vertex.activate();
									//System.out.println(vertex.is_activated_curr()+","+vertex.is_activated_next()+","+vertex.is_capable());

									//influence++;
								}						
							}					
						}			
					}			
				}
	
				//MAKE newly activated vertices inactive
				for (Vertex vertex : socialNetwork.getVertexList().values()) {			
					if (vertex.is_capable()) vertex.makeIncapable(); // its neighbor becomes inactive once acts	
				}		
	
			}
	
			return influence;
		}

	
	public int getInfluence_v2(HashSet <V> active_nodes){
		int influence=active_nodes.size();

		//System.out.println("~~~~~~~~Stage:0~~~~~~~~~");
		//System.out.println("Seeds:"+active_nodes.toString());

		//ASSIGN states of all vertices according the list of active nodes
		for (Vertex vertex : socialNetwork.getVertexList().values()) {			
			if(active_nodes.contains(vertex.getID())) {
				vertex.activate();
			}
			else vertex.reset();
		}		

		//IMPLEMENT the independent cascade process
		for (int t=0; t<this.MAX_STAGES; t++){	
			//System.out.println("ITR:"+t);
			
			ArrayList <Vertex> actors=new ArrayList(); //List of active nodes that are capable to active neighbors

			//UPDATE states stage by stage discretely
			for (Vertex vertex : socialNetwork.getVertexList().values()) {			
				vertex.updateState();
				if(vertex.is_capable()) actors.add(vertex);
			}
			
			//System.out.println("ACTr:"+actors);


			//IF NO MORE CAPABLE NODES: BREAK
			if(actors.size()==0) break;
			
			//System.out.println("~~~~~~~~Stage:"+(t+1)+"~~~~~~~~~");
			for(Vertex actor:actors){ //each actor activates its neighbors
				//System.out.println("Current actor:"+actor.getID());

				
				ArrayList <V> nb_IDs= socialNetwork.getAdjacentVertices((V) actor.getID());//Neighbors

				for (V nb_id: nb_IDs){ //for each adjacent vertex
					//System.out.println(">>Neighbor:"+nb_id);

					Vertex nb=socialNetwork.getVertexList().get(nb_id); //get actual node object thru its id
					//System.out.println("is_capable:"+v.is_capable());
					//System.out.println(nb.is_activated_curr()+","+nb.is_activated_next()+","+nb.is_capable());					

					if(!nb.is_activated_curr()&&!nb.is_activated_next()){

						double prob;// = socialNetwork.getDistanceBetween(v_id, (V) vertex.getID()); //get edge weight, i.e. activation probability
						//NOTE: A FAST METHOD/representation may be needed to get weight...
						//TEST ONLY prob=0.5, 0.8
						prob=0.1;

						//Get activated at a chance determined by the weight
						Random rn = new Random(System.nanoTime());
						double rn_db=rn.nextDouble();
						//System.out.println("Random Draw:"+rn_db);

						if (rn_db<=prob) {
							//System.out.println("ACTIVATED!!!");
							influence++; 
							nb.activate();
							//System.out.println(nb.is_activated_curr()+","+nb.is_activated_next()+","+nb.is_capable());
						}						
					}										
				}
			}		

			//MAKE newly activated vertices inactive
			for (Vertex actor:actors) {			
				actor.makeIncapable(); // its neighbor becomes inactive once acts
				//System.out.println(actor);
			}		

		}
	
		return influence;
	}
	/**
	 * Compute expected influence of multiple model runs
	 * @param active_nodes active nodes initially specified
	 * @return expected influence of multiple model runs
	 */
	public double getExpectedInfluence(HashSet <V> active_nodes){
		double expectedInfluence=0;
		HashSet <V> social_active_nodes=this.filter(active_nodes,socialNetwork);
		//System.out.println(social_active_nodes);
		for (int r=0; r<this.MAX_RUNS; r++){
			int delta_inf=this.getInfluence_v2(social_active_nodes);
			//System.out.println(r+":"+delta_inf);
			expectedInfluence+=delta_inf;		
		}
		
		//System.out.println(active_nodes);
		return expectedInfluence/this.MAX_RUNS;		
	}

	/**
	 * Filter out nodes which are not socially connected, i.e., way points but not houses
	 * @param active_nodes 
	 * @param socialNet
	 * @return nodes in social network
	 */
	private HashSet<V> filter(HashSet<V> active_nodes, Graph<V> socialNet) {
		HashSet<V> social_nodes=new HashSet<V>();

		for (V v: active_nodes){
			if (socialNet.getVertexList().containsKey(v)) 
				social_nodes.add(v);			
		}
		return social_nodes;
	}

	public Graph<V> getSocialNetWork(){
		return 	socialNetwork;	
	}

}
