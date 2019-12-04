package submodularMaxOverGraph;

import java.util.HashSet;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;

/**
 * Class of common methods in mobile sensor application
 * @author zhangh24
 *
 */
public class MobileSensorUtil {

	public MobileSensorUtil() {
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * Compute conditional entropy
	 * @param cov
	 * @param v
	 * @param x
	 * @return
	 */
	
	public static <V> double getConditionalEntropy(DoubleMatrix cov, V v, HashSet<V> x){
		if(x.contains(v)) return 0; //Bug fixed 1.4.2016
		//System.out.println("v:"+v);
		//System.out.println("x:"+x);
		double conditionalEntropy=0;
		double K_yy=subMatrix(cov, v, v).get(0,0);
		//System.out.println("K_yy:"+K_yy);
		DoubleMatrix K_yA=(x.size()>0)?subMatrix(cov, v, x):new DoubleMatrix(1,1,0);
		//System.out.println("K_yA:"+K_yA);
		DoubleMatrix K_AA_ivs=(x.size()>0)?Solve.pinv(subMatrix(cov, x, x)):new DoubleMatrix(1,1,0);
		//System.out.println("K_AA_ivs:"+K_AA_ivs);
		double delta_f=K_yy-
				(K_yA.mmul(K_AA_ivs).mmul(K_yA.transpose())).get(0,0); //utility changes
		//System.out.println("K_yA.mmul(K_AA_ivs).mmul(K_yA.transpose()):"+K_yA.mmul(K_AA_ivs).mmul(K_yA.transpose()));
		
		//Convert to entropy instead of conditional variance
		//System.out.println("delta_f:"+delta_f);
		conditionalEntropy=0.5*Math.log(2*Math.PI*Math.E*delta_f);		
		return conditionalEntropy;
	}	
	
	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, HashSet<V> list_row, HashSet<V> list_col) {
		DoubleMatrix sub_matrix = null;
		int[] list_row_num=new int[list_row.size()];
		int[] list_col_num=new int[list_col.size()];				

		int i=0;

		for(V v: list_row){
			int idx=Integer.valueOf((String) v)-1001;
			list_row_num[i]=idx;
			i++;
		}

		i=0;
		for(V v: list_col){
			int idx=Integer.valueOf((String) v)-1001;
			list_col_num[i]=idx;
			i++;
		}

		//1 GET rows
		sub_matrix=matrix.getRows(list_row_num);				
		//2 GET cols	
		sub_matrix=sub_matrix.getColumns(list_col_num);

		return sub_matrix;		
	}

	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, V v1, V v2) {
		HashSet s_v1=new HashSet();
		HashSet s_v2=new HashSet();

		s_v1.add(v1);
		s_v2.add(v2);

		return subMatrix(matrix, s_v1, s_v2);		
	}

	public static <V> DoubleMatrix subMatrix(DoubleMatrix matrix, V v, HashSet<V> list_col) {
		HashSet s_v=new HashSet();
		s_v.add(v);
		return subMatrix(matrix, s_v, list_col);

	}
}
