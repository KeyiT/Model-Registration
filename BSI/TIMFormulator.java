package artisynth.models.swallowingRegistrationTool.BSI;

import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.correspondences.TrivariateInjectiveMap;
import artisynth.models.swallowingRegistrationTool.optimizers.PointInjectiveMapFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.MatrixNd;

public class TIMFormulator implements PointInjectiveMapFormulator{
   
   TrivariateInjectiveMap myMap;

   @Override
   public MatrixNd makeSourceDataMatrix () {
      return myMap.makeSourceDataMatrix ();
   }

   @Override
   public MatrixNd makeTargetDataMatrix () {
      return myMap.makeTargetDataMatrix ();
   }

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      // hook here, subclass can override
   }

   @Override
   public void setCloudMap (CloudMap map) {
      if (map instanceof TrivariateInjectiveMap) {
         myMap = (TrivariateInjectiveMap)map;
         return;
      }
      
      throw new IllegalArgumentException ("Incompatible map");
   }

   @Override
   public MatrixNd makeTargetedChangeMatrix () {
      MatrixNd Change = new MatrixNd ();
      Change.sub (myMap.makeTargetDataMatrix (), 
         myMap.makeSourceDataMatrix ());
      return Change;
   }
}
