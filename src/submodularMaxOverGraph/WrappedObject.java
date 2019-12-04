package submodularMaxOverGraph;

import java.util.HashMap;

public class WrappedObject <V> {

	public HashMap <V, Double> dist;
	public HashMap<V, V> prev;
	
	public WrappedObject(HashMap <V, Double> min_dist,HashMap<V, V> prev_node ) {
		dist=min_dist;
		prev=prev_node;		
	}
}
