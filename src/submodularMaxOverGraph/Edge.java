package submodularMaxOverGraph;
public class Edge<V> implements Comparable <Edge <V>>{
	private V vertex;
	private V fromVertex;
	private boolean covered;

	//Road network
	private double length;

	//Social network
	private double activate_prob; 

	public Edge(V vert, double weight2) {
		vertex = vert;
		length = weight2;
		covered=false;
	}

	public Edge(V from, V to, double weight) {
		fromVertex=from;
		vertex = to;
		length = weight;
		covered=false;
	}

	public boolean isCovered(){
		return this.covered;
	}
	
	public boolean setCovered(){
		return this.covered=true;
	}
	
	public V getVertex() {
		return vertex;
	}
	public void setVertex(V vertex) {
		this.vertex = vertex;
	}
	public V getFromVertex(){
		return this.fromVertex;
	}
	
	public double getWeight() {
		return length;
	}
	public void setWeight(int weight) {
		this.length = weight;
	}
	public String toString(){
		return "( "+ fromVertex+"->"+vertex + ", " + length + ","+covered+" )";
	}

	@Override
	public int compareTo(Edge<V> o) {
		return Double.compare(length, o.length);	
	}
	
	public void setFromVertex(V from) {
		fromVertex = from;
	}	
}
