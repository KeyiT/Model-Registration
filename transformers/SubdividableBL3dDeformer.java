package artisynth.models.swallowingRegistrationTool.transformers;

import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public interface SubdividableBL3dDeformer {
   
   public void setWorkingSubdomain (int idx);
   
   public int getWorkingSubdomain ();
   
   public int [][] findSubdomains (MatrixNd pntsInWorld);
   
   public Matrix makeSubdomainBasis (MatrixNd data);
   
   public void getSubdomainCoef (VectorNd Coef);
   
   public void setSubdomainCoef (VectorNd Coef);
   
   public void enableSubdivision (boolean enable);
   
   public boolean isSubdivisionEnabled ();
}
