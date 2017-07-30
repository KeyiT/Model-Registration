package artisynth.models.swallowingRegistrationTool.infoUtilities;

import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;

public class VertexInfo extends ObjInfo {

   public Vertex3d myVertex;
   MeshBase myParentMesh;
   
   public VertexInfo (Vertex3d vtx) {
      setName("VtxInfo");
      myVertex = vtx;
   }
   
   public VertexInfo (String name) {
      setName(name);
   }
   
   public VertexInfo (String name, Vertex3d vtx) {
      this (name);
      myVertex = vtx;
   }
   
   public void setParentMesh (MeshBase mesh) {
      myParentMesh  = mesh;
   }
   
   public MeshBase getParentMesh () {
      return myParentMesh;
   }
   

   @Override
   public VertexInfo clone () throws CloneNotSupportedException {
      @SuppressWarnings("unchecked")
      VertexInfo v = new VertexInfo(getName());
      v.myVertex = myVertex;
      v.setParentMesh (myParentMesh);
      return v;
   }



}
