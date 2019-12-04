package submodularMaxOverGraph;

import java.util.ArrayList;
import java.util.HashMap;

public class WrapWalkWithLength<V> {

	public ArrayList <V> walk;
	public double length;	
	

	public WrapWalkWithLength(ArrayList<V> return_walk, double walk_len) {
		walk=return_walk;
		length=walk_len;
	}

}
