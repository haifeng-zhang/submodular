package submodularMaxOverGraph;

public class Vertex <V> implements Comparable <Vertex <V>> {
	private V v_id;//Vertex ID

	//Road network
	private double visit_cost;
	private boolean is_visited;
	private double x_coord, y_coord;

	//Dijkstra algorithm related fields
	public double min_dist; //minimum distance from source to current vertex
	public V pre_node; //ID of predecessor vertex

	//Social network 	
	private boolean is_activated_curr; //current state: whether if it is activated, i.e., either seeded or affected by neighbors 
	private boolean is_activated_next; //next state: whether if it is activated, i.e., either seeded or affected by neighbors 
	private boolean is_capable; //whether if it is capable to affect neighbors, i.e., capable only during the stage when it is initially activated

	public Vertex(V id, double vc) {
		v_id = id;
		visit_cost = vc;
		is_visited=false;
	}

	public void setCoordinates(double x, double y){
		x_coord=x;
		y_coord=y;
	}

	/**
	 * Reset state
	 */
	public void reset(){
		is_activated_next=false;
		is_capable=false;
	}

	/**
	 * Make a node incapable to affect its neighbors
	 */
	public void makeIncapable(){
		is_capable=false;		
	}

	/**
	 * Become activated in next stage
	 */
	public void activate(){
		is_activated_next=true;	
		is_capable=true;
	}

	/**
	 * Get its capability state
	 * @return capability state
	 */
	public boolean is_capable(){
		return is_capable&&is_activated_curr;		
	}

	/**
	 * Get current state of activeness 
	 * @return current state of activeness
	 */
	public boolean is_activated_curr(){
		return is_activated_curr;		
	}

	/**
	 * Get next state of activeness 
	 * @return next state of activeness
	 */
	public boolean is_activated_next(){
		return is_activated_next;		
	}
	
	
	/**
	 * Update state of activeness
	 */
	public void updateState(){
		is_activated_curr=is_activated_next;
		//is_activated_next=false;
	}


	public String toString() {
		String s = "";
		s += v_id.toString();
		s += " : ";
		s += "visit_cost="+visit_cost+",";
		//s += "x_coord="+x_coord+", y_coord="+y_coord+",";
		//s += "is_visited="+is_visited;
		s += "\n";

		return s;
	}

	public int compareTo(Vertex<V> other) {
		return Double.compare(min_dist, other.min_dist);
	}

	public V getID (){
		return v_id;		
	}


}
