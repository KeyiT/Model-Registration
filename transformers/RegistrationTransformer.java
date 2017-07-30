package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.ObjInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.TransformerSlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.OptimizerSink;
import maspack.geometry.GeometryTransformer;
import maspack.geometry.MeshBase;

/**
 * 
 * @author KeyiTang
 *
 */
public abstract class RegistrationTransformer extends GeometryTransformer
implements OptimizerSink{
   
   Map<TransformableGeometry, SlaveInfo> mySlaves = 
   new LinkedHashMap<TransformableGeometry, SlaveInfo>();
   
   protected ArrayList <MeshBase> myMeshes = new ArrayList<MeshBase> ();
   
   protected boolean isFast = false;
   
   public void addMasterMesh (MeshBase master) {
      if (containMasterMesh (master)) {
         return;
      }
      myMeshes.add (master);
   }
   
   public boolean containMasterMesh (MeshBase master) {
      if (myMeshes.contains (master)) {
         return true;
      }
      else {
         return false;
      }
   }
   
   public void addMasterMeshes (Collection <MeshBase> meshes) {
      for (MeshBase mesh : meshes) {
         if (myMeshes.contains (mesh)) {
            continue;
         }
         myMeshes.add (mesh);
      }
   }
   
   public void removeMasterMesh (MeshBase master) {
      myMeshes.remove (master);
   }
   
   public abstract TransformerSlaveInfo createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info);
   
   /**
    * add new slave to this transformer; If this slave has been added to this
    * transformer, old <code>SlaveInfo</code> stored in hash map will be replaced;
    * This method will create a <code>TransformerSlaveInfo</code> for <tt>slave</tt>
    * and add into <tt>info</tt>.
    * 
    * @param info <code>SlaveInfo<code> of slave
    * @return old <code>SlaveInfo</code> associated with <tt>slave</tt>
    */
   public SlaveInfo addSlave (SlaveInfo info) {
      SlaveInfo oldInfo = mySlaves.put (info.getSlave (), info);
      if (oldInfo != null) {
         System.out.println ("warning: old slave info been replaced!");
      }
      
      info.addTransformerInfo (this);
      return oldInfo;
   }
   
   /**
    * put slave into this transformer; If this slave has been added to this
    * transformer, old <code>SlaveInfo</code> stored in hash map will be replaced;
    * 
    * 
    * @param info <code>SlaveInfo<code> of slave
    * @return old <code>SlaveInfo</code> associated with <tt>slave</tt>
    */
   public SlaveInfo putSlave (SlaveInfo info) {
      SlaveInfo oldInfo = mySlaves.put (info.getSlave (), info);
      if (oldInfo != null) {
         System.out.println ("warning: old slave info been replaced!");
      }
      return oldInfo;
   }
   
   /**
    * remove slave from this transformer, it will no influence <tt>info</tt>
    * content
    * @param info
    */
   public void removeSlave (SlaveInfo info) {
      mySlaves.remove (info.getSlave ());
   }
   
   public boolean isMySlave (SlaveInfo info) {
      return mySlaves.containsValue (info);
   }
   
   public boolean isMySlave (TransformableGeometry slave) {
      return mySlaves.containsKey (slave);
   }
   
   /**
    * 
    * @param info
    * @return if <tt>info</tt> contains <code>TransformerSlaveInfo</code>
    * of this transformer return true, otherwise return false;
    */
   public boolean isKnownSlave (SlaveInfo info) {
      TransformerSlaveInfo myInfo = info.getTransformerInfo (this);
      if (myInfo != null) {
         return true;
      }
      return false;
   }
   
   /**
    * Substitute old slaves to new slaves, if any new slaves do not 
    * contain info of this transformer it will generate 
    * <code>TransformerSlaveInfo<code> for it, otherwise just put them.
    * @param newSlaves
    */
   public void shiftSlaves (Collection<SlaveInfo> newSlaves) {
      mySlaves.clear ();
      for (SlaveInfo info : newSlaves) {
         if (isKnownSlave(info)) {
            putSlave (info);
         }
         else {
            addSlave (info);
         }
      }
   }
   
   /**
    * Append slaves to the <tt>slaves</tt> list;
    * @param slaves
    */
   public void getSlaves(List<TransformableGeometry> slaves) {
      Set entrySet = mySlaves.entrySet ();
      Iterator it = entrySet.iterator ();
      
      while (it.hasNext ()) {
         Entry<TransformableGeometry, SlaveInfo> me = 
         (Entry<TransformableGeometry, SlaveInfo>)it.next ();
         
         TransformableGeometry slave = me.getKey ();
         slaves.add (slave);
      }
   }

   public SlaveInfo getSlaveInfo (TransformableGeometry slave) {
      return mySlaves.get (slave);
   }
   
   public void getSlaveInfos (List<SlaveInfo> list) {
      Set entrySet = mySlaves.entrySet ();
      Iterator it = entrySet.iterator ();
      
      while (it.hasNext ()) {
         Entry<TransformableGeometry, SlaveInfo> me = 
         (Entry<TransformableGeometry, SlaveInfo>)it.next ();
         
         SlaveInfo info = me.getValue ();
         list.add (info);
      }
   }
   
   public boolean updateSlaveInfos () {
      Set entrySet = mySlaves.entrySet ();
      Iterator it = entrySet.iterator ();
      
      boolean updated = true;
      
      while (it.hasNext ()) {
         Entry<TransformableGeometry, SlaveInfo> me = 
         (Entry<TransformableGeometry, SlaveInfo>)it.next ();
         
         SlaveInfo info = me.getValue ();
         if (info != null) {
            updated &= info.update (this);
         }
      }
      return updated;
   }
   
   public boolean update () {
      return updateSlaveInfos ();
   }

   public abstract void applyAction ();
   
   public abstract void resetAction ();
   
   
   /**
    * Enable fast transformation;
    * @see #fastTransformer
    * @param enable
    */
   public void enableFastTranform (boolean enable) {
      isFast = enable;
   }
   
   /**
    * @see #fastTransformer
    * 
    * @return if this is a fast transformer return true, otherwise 
    * return false; A false value will enforce slaves transformed in 
    * standard way;
    */
   public boolean isFastTransformer () {
      return isFast;
   } 
   
   /**
    * fast way to transformer geometries; If this is a fast transformer,
    * this method will be called by <code>RegistrationManager</code> in
    * iteration action to transform active slaves; 
    * 
    * @see #isFastTransformer
    * 
    * @param slaves slaves need to be transformed; this method should remove
    * the slaves that are transformed successfully, and remain unsupported
    * slaves in the set which will be transformed in the standard way;
    * 
    */
   public abstract void fastTransformer (
      Set<SlaveInfo> slaves);
   
}
