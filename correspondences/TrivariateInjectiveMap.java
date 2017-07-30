package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.Iterator;
import java.util.Set;
import java.util.Map.Entry;

import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;

public abstract class TrivariateInjectiveMap<T extends Correspondence> extends 
InjectiveMap<Vector3d, Vector3d, T> {
   
   public TrivariateInjectiveMap () {
      isAllocatable = false;
      super.setSourceDataDimension (3);
      super.setTargetDataDimension (3);
   }
   
   public void setSourceDataDimension (int Dimension) {
      throw new UnsupportedOperationException (
         "Fixed data dimension!");
   }
   
   public void setTargetDataDimension (int Dimension) {
      throw new UnsupportedOperationException (
      "Fixed data dimension!");
   }

   
   /**
    * {@inheritDoc}
    */
   @Override
   public MatrixNd makeTargetedChangeMatrix () {
      if (myMap.size () == 0) {
         throw new ImproperStateException(
            "No map data!");
      }
      MatrixNd tgtX = new MatrixNd (myMap.keySet ().size (), 3);
      int idx = 0; 
      
      Set <Entry<Vector3d, Vector3d>> entry = myMap.entrySet ();
      Iterator<Entry<Vector3d, Vector3d>> iter = entry.iterator ();
      Vector3d dis = new Vector3d ();
      while (iter.hasNext ()) {
         Entry<Vector3d, Vector3d> me = iter.next ();
         dis.sub (me.getValue (), me.getKey ());
         tgtX.setRow (idx++, dis);
      }

      return tgtX;
   }
   
   /**
    * 
    * old partner associate with self in hash map 
    * will be replaced by the new partner of 
    * <tt>feature</tt>
    * @param feature 
    * @return old partner associate with self in hash map
    */
   @Override
   public Vector3d addInjectiveMap (HasPartner correspondence) {
      if ( !(correspondence instanceof Point3d2Point3d)) {
         throw new IllegalArgumentException (
            "Incompatible correspondence");
      }
      Point3d2Point3d feature = (Point3d2Point3d) correspondence;
      if (!assignPartners (feature)) {
         return null;
      }
      Vector3d partner = feature.getPartner ();
      Vector3d self = feature.getSelf ();
      return myMap.put (self, partner);
   }
   
   /**
    * specify <tt>self</tt> and <tt>partner</tt> in <tt>correspondence</tt>
    * @param correspondence
    * @return a value of false will skip the <tt>correspondence</tt> when update 
    * injective map;
    */
   public abstract boolean assignPartners (Point3d2Point3d correspondence);
   
}
