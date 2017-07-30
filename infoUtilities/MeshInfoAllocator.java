package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import artisynth.models.swallowingRegistrationTool.correspondences.Correspondence;
import maspack.geometry.MeshBase;

public abstract class MeshInfoAllocator implements InfoAllocator <Correspondence>{

   protected Set <MeshBase> myMeshSet = new LinkedHashSet<MeshBase>();
   
   protected MeshFeature myMeshFeature = MeshFeature.Vertex;
   
   public enum MeshFeature {
      Vertex,
      Edge,
      Face
   }
   
   public MeshInfoAllocator () {
      // do nothing
   }
   
   public MeshInfoAllocator (Collection<? extends MeshBase> meshes) {
      myMeshSet.addAll (meshes);
   }
   
   public void setAllocateFeature (MeshFeature feat) {
      myMeshFeature = feat;
   }
   
   public MeshFeature getAllocateFeature () {
      return myMeshFeature;
   }
   
   /**
    * Add a mesh to this allocator
    *
    * @param mesh to be added to this allocator
    * @return <tt>true</tt> if this set did not already contain the specified
    * element; no duplicated mesh is allowed in this allocator.
    */
   public boolean addMesh (MeshBase mesh) {
      return myMeshSet.add (mesh);
   }
   
   public void addMeshes (Collection<MeshBase> meshes) {
      myMeshSet.addAll (meshes);
   }
   
   /**
    * Retrieve meshes in this allocator and append them into <tt>list</tt>;
    * @param list 
    */
   public void getMeshes (List<MeshBase> list) {
      Iterator it = myMeshSet.iterator ();
      while (it.hasNext ()) {
         MeshBase mesh = (MeshBase)it.next ();
         list.add (mesh);
      }
   }
   
   /**
    * Test if <tt>mesh</tt> in this allocator;
    * @param mesh mesh to be test
    * @return if <tt>mesh</tt> was in this allocator return <tt>true</tt>;
    */
   public boolean containMesh (MeshBase mesh) {
      return myMeshSet.contains (mesh);
   }
   
   public void removeMesh (MeshBase mesh) {
      if (myMeshSet.contains (mesh)) {
         myMeshSet.remove (mesh);
      }
      else {
         throw new NullPointerException ("Mesh not in this allocator!");
      }
   }
   
   /**
    * Remove all meshes in this allocator
    */
   public void clearMeshAllocator () {
      myMeshSet.clear ();
   }
   

   
   

}
