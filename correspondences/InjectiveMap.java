package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import artisynth.models.swallowingRegistrationTool.optimizers.PointInjectiveMapFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

import java.util.Map.Entry;

import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

public abstract class InjectiveMap<K extends Vector, V extends Vector, T extends Correspondence> 
extends CloudMap<T> implements PointInjectiveMapFormulator{
   
   LinkedHashMap <K, V> myMap = new LinkedHashMap <K, V> (); 
   private int srcDim = 0;
   private int tgtDim = 0;

   private VectorNd myWeights = new VectorNd ();
   private boolean weighted = false;
   
   /**
    * <tt>Correspondence</tt> must has dynamic partners
    */
   @Override
   public boolean addInfo (T correspondence) {
      if (correspondence instanceof HasPartner) {
         if (super.addInfo (correspondence)) {
            return true;
         } 
         return false;
      }
      else {
         System.err.println (
            "Incompatible correspondence!");
         return false;
      }
   }

   /**
    * add a new correspondence map to hash set. 
    * If a duplicate self(key) has existed in hash
    * set, old partner(value) will be replaced by 
    * the new partner. 
    * 
    * @param feature : single correspondence which 
    * has dynamic map.
    * @return old partner or null
    */
   abstract public V addInjectiveMap (HasPartner feature);
   
   /**
    * modification over returned map will change this
    * injective map.
    * @return
    */
   public Map<K, V> getInjectiveMap () {
      return myMap;
   }
   
   /**
    * 
    * @param rmap injective vector map will be returned 
    * in <tt>rmap</tt>
    */
   public void getInjectiveMap (Map <Vector, Vector> rmap) {
      Set ens = myMap.entrySet ();
      Iterator it=  ens.iterator ();
      while (it.hasNext ()) {
         Entry<Vector, Vector> me = (Entry<Vector, Vector>)it.next ();
         rmap.put (me.getKey (), me.getValue ());
      }
   }
   
   /**
    * add a new correspondence to map directly.
    * If a duplicate self(key) has existed in hash
    * set, old partner(value) will be replaced by 
    * the new partner. 
    * 
    * @return old partner or null
    */
   public V addInjectiveMap (K key, V val) {
      if (key.size () != srcDim) {
         throw new IllegalArgumentException (
            "Incompatible source data dimension");
      }
      if (val.size () != tgtDim) {
         throw new IllegalArgumentException (
            "Incompatible target data dimension");
      }
      return myMap.put (key, val);
   }
   
   /**
    * clear correspondence hash map
    */
   public void clearInjectiveMap () {
      myMap.clear ();
   }
   
   /**
    * remove a correspondence whose key is <tt>self</tt>;
    * the partner(value) will be returned.
    * 
    * @param self
    * @return partner
    */
   public V removeInjectiveMap (K self) {
      return myMap.remove (self);
   }
   
   /**
    * Any subclass should assign a weight for each map. 
    * 
    * This method is to retrieve the mapping weights <tt>W_n</tt> which is defined 
    * as a (<tt>N</tt>) elements vector. 
    * 
    * @return mapping weights matrix <tt>W_n</tt>.
    */
   public VectorNd getMapWeights () {
      return myWeights;
   }
   
   /**
    * enable mapping weight;
    * @param enable if false every map is treated in the same manner
    */
   public void enableMapWeight (boolean enable) {
      weighted = enable;
   }
   
   /**
    * @return if different correspondences have different 
    * weight return true, otherwise return false
    */
   public boolean mapWeightEnabled () {
      return weighted;
   }
   
   /**
    * set weight vector size to zero
    */
   public void clearMapWeights () {
      myWeights.setSize (0);
   }
   
   
   public void setSourceDataDimension (int Dimension) {
      srcDim = Dimension;
   }
   
   public void setTargetDataDimension (int Dimension) {
      tgtDim = Dimension;
   }
   
   public int getSourceDataDimension () {
      return srcDim;
   }
   
   public int getTargetDataDimension () {
      return tgtDim;
   }
   
   /**
    * if source data dimension is the same as target data dimension
    * return true otherwise return false;
    */
   public boolean isCompatibleDimension () {
      if (srcDim == tgtDim) {
         return true;
      }
      return false;
   }

   /**
    * {@inheritDoc}
    * 
    */
   @Override
   public MatrixNd makeSourceDataMatrix () {
      if (myMap.size () == 0) {
         throw new ImproperStateException(
            "No map data!");
      }
      MatrixNd srcX = new MatrixNd (myMap.size (), srcDim);
      int idx = 0; 
      
      Set entry = myMap.entrySet ();
      Iterator iter = entry.iterator ();
      while (iter.hasNext ()) {
         Entry<K, V> me = (Entry<K, V>)iter.next ();
         Vector key = me.getKey ();
         if (key.size () != srcDim) {
            throw new ImproperSizeException (
               "Incompatible data dimension");
         }
         srcX.setRow (idx++, key);
      }

      return srcX;
   }
   
   /**
    * {@inheritDoc}
    * 
    */
   @Override
   public MatrixNd makeTargetDataMatrix () {
      if (myMap.size () == 0) {
         throw new ImproperStateException(
            "No map data!");
      }
      MatrixNd tgtX = new MatrixNd (myMap.size (), tgtDim);
      int idx = 0; 
      
      Set entry = myMap.entrySet ();
      Iterator iter = entry.iterator ();
      while (iter.hasNext ()) {
         Entry<K, V> me = (Entry<K, V>)iter.next ();
         Vector val = me.getValue ();
         if (val.size () != tgtDim) {
            throw new ImproperSizeException (
               "Incompatible data dimension");
         }
         tgtX.setRow (idx++, val);
      }

      return tgtX;
   }
   
   public int numInjectiveMaps () {
      return myMap.size ();
   }
   
   @Override
   public void setTransformer (RegistrationTransformer tf) {
      // do nothing
   }

   @Override
   public void setCloudMap (CloudMap map) {
      // do nothing
   }
   
   
   public boolean update () {
      boolean updated = super.update ();
      if (! updated) {
         throw new ImproperStateException (
            "Failed to update the injective map");
      }
      
      myMap.clear ();
      List<T> list = new ArrayList<T>();
      getInfos (list);
      for (T feature : list) {
         HasPartner me = (HasPartner) feature;
         addInjectiveMap (me);
      }
      
      return true;
   }
   
}
