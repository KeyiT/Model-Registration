package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.InternalErrorException;

public class WeightsMaker {

   /**
    * Generate weights for each vertices mapping pair. Weights are generated
    * with guassian function, whose input is negative mean ratio of the nearest  
    * FEM element, output range from 0.8 to 1.0(standard deviation is 3.0); Low 
    * mean ratio corresponds to low weight, and vice verser;
    * 
    * @param fem
    * @return
    */
   public static VectorNd createFEMQualityBasedGuassianWeights (
      FemModel3d fem, Map myMap) {
      
      VectorNd weights = new VectorNd (myMap.size ());
      Set<Map.Entry<Vector3d, Vector3d>> entry = myMap.entrySet ();
      FEMQualityUtilities QU = new FEMQualityUtilities ();
      
      Iterator It = entry.iterator ();
      int idx = 0;
      double eta = 0;
      while (It.hasNext ()) {
         Map.Entry<Vector3d,Vector3d> me = (Map.Entry<Vector3d,Vector3d>)It.next ();
         Point3d key = new Point3d(me.getKey ());
         FemNode3d node = fem.findNearestNode (key, 1E-5);
         if (node != null) {
            LinkedList<FemElement3d> eles = node.getElementDependencies ();
            VectorNd etas = new VectorNd(eles.size ());
            Iterator<FemElement3d> ite = eles.iterator ();
            for (int i = 0; i < etas.size (); i++) {
               etas.set (i, QU.evalMeanRatio (ite.next ()));
            }
            eta = etas.minElement ();
         }
         else {
            try {
            FemElement3d ele = fem.findNearestSurfaceElement (
               null, new Point3d(key));
            eta = QU.evalMeanRatio (ele);
            }
            catch (InternalErrorException e) {
               eta = 1.0;
            }
         }
         // from 0.8 to 1.0
         double w = evalGuassianWeight (-eta, 3.0);
         weights.set (idx++, w);
      }
      return weights;
   }
   
   private static double evalGuassianWeight (double x, double sig) {
      if (sig == 0) {
         if (x == 0) {
            return 1.0;
         }
         else {
            return 0;
         }
      }
      double w = Math.exp ( - (x * x) / 2.0 / (sig * sig) );
      return w;
   }

}
