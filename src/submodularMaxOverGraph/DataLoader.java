package submodularMaxOverGraph;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.jblas.DoubleMatrix;

public class DataLoader {
	private static final String COMMA_DELIMITER = ",";

	public DataLoader() {
		// TODO Auto-generated constructor stub
	}

	public static void loadNetwork(Graph graph, boolean directed, String edgeListFile, String nodeListFile) {
		BufferedReader fileReader = null;


		//STEP 1: ADD edges
		try {
			String line = "";

			//Create the file reader
			fileReader = new BufferedReader(new FileReader(edgeListFile));

			//Read the CSV file header to skip it
			fileReader.readLine();

			//Read the file line by line starting from the second line
			while ((line = fileReader.readLine()) != null) {
				//Get all tokens available in line
				String[] tokens = line.split(COMMA_DELIMITER);
				String from_node=tokens[0];			
				String to_node=tokens[1];	
				double weight=Double.parseDouble(tokens[2]);				

				if (!directed)
					graph.addEdge(from_node, to_node, weight);
				else graph.addArc(from_node, to_node, weight);
			}

		}
		catch (Exception e) {
			System.out.println("Error while reading CSV file !!!");
			e.printStackTrace();
		} finally {
			try {
				fileReader.close();
			} catch (IOException e) {
				System.out.println("Error while closing CSV file !!!");
				e.printStackTrace();
			}
		}

		//STEP 2: ASSIGN values to nodes
		try {
			String line = "";

			//Create the file reader
			fileReader = new BufferedReader(new FileReader(nodeListFile));

			//Read the CSV file header to skip it
			fileReader.readLine();

			//Read the file line by line starting from the second line
			while ((line = fileReader.readLine()) != null) {
				//Get all tokens available in line
				String[] tokens = line.split(COMMA_DELIMITER);
				String nodeID=tokens[0];			
				double x_coord=Double.valueOf(tokens[1]);	
				double y_coord=Double.valueOf(tokens[2]);				

				Vertex v=(Vertex) (graph.getVertexList().get(nodeID));
				if (v!=null) v.setCoordinates(x_coord, y_coord);
				else graph.add(nodeID, new ArrayList<Edge<String>> ());
			}

		}
		catch (Exception e) {
			System.out.println("Error while reading CSV file !!!");
			e.printStackTrace();
		} finally {
			try {
				fileReader.close();
			} catch (IOException e) {
				System.out.println("Error while closing CSV file !!!");
				e.printStackTrace();
			}
		}

	}

	public static void loadMatrix(DoubleMatrix readto, String matrixFileName) {
		BufferedReader fileReader = null;
		
		//READ TO matrix
		try {
			String line = "";

			//Create the file reader
			fileReader = new BufferedReader(new FileReader(matrixFileName));

			//SKIP header line
			fileReader.readLine();

			int row=0;
			
			//Read the file line by line starting from the second line
			while ((line = fileReader.readLine()) != null) {
				//Get all tokens available in line
				String[] tokens = line.split(COMMA_DELIMITER);
				
				
				int col=0;

				for(String s:tokens){
					double val=Double.parseDouble(s);
					readto.put(row, col, val); 
					col++;
				}
				
				row++;

			}

		}
		catch (Exception e) {
			System.out.println("Error while reading CSV file !!!");
			e.printStackTrace();
		} finally {
			try {
				fileReader.close();
			} catch (IOException e) {
				System.out.println("Error while closing CSV file !!!");
				e.printStackTrace();
			}
		}

	}	
}
