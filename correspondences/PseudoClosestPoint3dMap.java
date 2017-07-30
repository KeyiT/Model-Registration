package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshICP;
import maspack.geometry.MeshICP.AlignmentType;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;

public class PseudoClosestPoint3dMap<C extends VertexToMeshCorrespondence> 
extends ClosestPoint3dMap<C>{
   
   Map <PolygonalMesh, PolygonalMesh> myMeshMap = 
   new LinkedHashMap<PolygonalMesh, PolygonalMesh> ();
   
   Map <PolygonalMesh, AffineTransform3d> myMeshTranMap = 
   new LinkedHashMap<PolygonalMesh, AffineTransform3d> ();
   
   Map <C, LockedInfo> myLocks = new LinkedHashMap <C, LockedInfo> ();
   
   private void addMeshMatchPair(PolygonalMesh src, PolygonalMesh tgt) { 
      myMeshMap.put (src, tgt);
   }
   
   public void getSourceMeshes (List<PolygonalMesh> srcs) {
      Set <Entry<PolygonalMesh, PolygonalMesh>> ens = myMeshMap.entrySet ();
      Iterator it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<PolygonalMesh,PolygonalMesh> me = 
         (Entry<PolygonalMesh,PolygonalMesh>)it.next ();
         srcs.add (me.getKey ());
      }
   }

   public void getTargetMeshes (List<PolygonalMesh> tgts) {
      Set <Entry<PolygonalMesh, PolygonalMesh>> ens = myMeshMap.entrySet ();
      Iterator it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<PolygonalMesh,PolygonalMesh> me = 
         (Entry<PolygonalMesh,PolygonalMesh>)it.next ();
         tgts.add (me.getValue ());
      }
   }
   
   public void buildLocks (AlignmentType alignType) {
      myMeshMap.clear ();
      myMeshTranMap.clear ();
      
      // build mesh map
      List<C> corrs = getInfos ();
      for (C corr : corrs) {
         MeshBase m1 = corr.getMesh ();
         MeshBase m2 = corr.getTargetMesh ();
         if (! (m1 instanceof PolygonalMesh)) {
            throw new UnsupportedOperationException (
               "Only support polygoanl mesh");
         }
         if (! (m2 instanceof PolygonalMesh)) {
            throw new UnsupportedOperationException (
               "Only support polygoanl mesh");
         }
         
         PolygonalMesh src;
         PolygonalMesh tgt;
         if (corr.getDirection ()) {
            src = (PolygonalMesh)m1;
            tgt = (PolygonalMesh)m2;
         }
         else {
            src = (PolygonalMesh)m2;
            tgt = (PolygonalMesh)m1;
         }
         if (!myMeshMap.containsKey (src)) {
            myMeshMap.put (src, tgt);
         }
         else {
            if (myMeshMap.get (src) != tgt) {
               throw new ImproperStateException (
                  "must be injective mesh map");
            }
         }
      }
      
      if (myMeshMap.size () <= 1) {
         return;
      }
      
      // fit
      Set <Entry<PolygonalMesh, PolygonalMesh>> ens = myMeshMap.entrySet ();
      Iterator it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<PolygonalMesh,PolygonalMesh> me = 
         (Entry<PolygonalMesh,PolygonalMesh>)it.next ();
         
         PolygonalMesh src = me.getKey ();
         PolygonalMesh tgt = me.getValue ();
         
         AffineTransform3d affine = new AffineTransform3d ();
         affine.set (MeshICP.align (src, tgt, alignType));
         myMeshTranMap.put (src, affine);
      }
      for (PolygonalMesh src : myMeshMap.keySet ()) {
         // transform back
         src.inverseTransform (myMeshTranMap.get (src));
      }
      for (C corr : corrs) {
         corr.update ();
      }
      
      // build lock
      for (PolygonalMesh src : myMeshMap.keySet ()) {
         // transform back
         src.transform (myMeshTranMap.get (src));
      }
      for (C corr : corrs) {
         if (corr.getDirection ()) {
            corr.setSelf (corr.getVertex ().getWorldPoint ());
            corr.setPartner (corr.getNearestPoint ());
         }
         else {
            PolygonalMesh src = (PolygonalMesh) 
            corr.getTargetMesh ();
            corr.getNearestPoint ().transform (myMeshTranMap.get (src));
            corr.setSelf (corr.getNearestPoint ());
            corr.setPartner (corr.getVertex ().getWorldPoint ());
            LockedInfo info = new LockedInfo ();
            info.setFace ((Face)corr.getNearestFeature ());
            info.setBary (corr.getBarycentric ());
            myLocks.put (corr, info);
         }
         corr.lockRelationship (true);
      }
   }
   
   
   
   private Set <MeshBase> LockedMesh = new HashSet<MeshBase> ();
   
   @Override
   public boolean assignPartners (Point3d2Point3d correspondence) {
      if (! (correspondence instanceof VertexToMeshCorrespondence)) {
         throw new IllegalArgumentException (
            "Incompatible correspondence");
      }

      VertexToMeshCorrespondence feature = 
      (VertexToMeshCorrespondence) correspondence;

      if (isMuted(feature)) {
         return false;
      }
      
      // re-lock
      if (myMeshMap.size () > 1) {
         if (feature.getDirection ()) {
            if (LockedMesh.contains (feature.getMesh ())) {
               feature.lockRelationship (true);
            }
            else {
               feature.lockRelationship (false);
            }
         }
         else {
            if (LockedMesh.contains (feature.getTargetMesh ())) {
               feature.lockRelationship (true);
            }
            else {
               feature.lockRelationship (false);
            }
         }
      }
      

      if (feature.isLocked ()) {
         if (myMeshMap.size () <= 1) {
            return true;
         }
         
         if (feature.getDirection ()) {
            feature.lockRelationship (false);
            feature.setSelf (feature.
               getVertex ().getWorldPoint ());
            feature.lockRelationship (true);
         }
         else {
            // barycentric -- > 3d point
            Point3d self = new Point3d ();
            LockedInfo info = myLocks.get (feature);
            info.getFace ().computePoint (self, info.getBary ());
            // local --> world
            if (!feature.getTargetMesh ().
            meshToWorldIsIdentity ()) {
               self.transform (feature.
                  getTargetMesh ().getMeshToWorld ());
            }
            feature.lockRelationship (false);
            feature.setSelf (self);
            feature.lockRelationship (true);
         }
         
         return true;
      }

      Point3d vtx = new Point3d (feature.getVertex ().
         getWorldPoint ());
      Point3d near = new Point3d (feature.
         getNearestPoint ());

      if (feature.getDirection ()) {
         feature.setSelf (vtx);
         feature.setPartner (near);
      }
      else {
         feature.setSelf (near);
         feature.setPartner (vtx);
         LockedInfo info = myLocks.get (feature);
         if (info != null) {
            info.setFace ((Face)feature.getNearestFeature ());
            info.setBary (feature.getBarycentric ());
         }
      }
      return true;
   }
   
   
   @Override
   public boolean update () {
      if (myMeshMap.size () > 1) {
         LockedMesh.clear ();
         for (PolygonalMesh srcMesh : myMeshMap.keySet ()) {
            if (!isClose (srcMesh, myMeshMap.get (srcMesh))) {
               LockedMesh.add (srcMesh);
            }
         }
      }
      return super.update ();
   }
   
   /**
    * 
    * @author KeyiTang
    *
    */
   protected class LockedInfo {
      Vector2d coor = new Vector2d ();
      Face face;
      
      public void setBary (Vector2d coor) {
         this.coor.set (coor);
      }
      
      public Vector2d getBary () {
         return coor;
      }
      
      public void setFace (Face face) {
         this.face = face;
      }
      
      public Face getFace () {
         return face;
      }
   }
   
   protected boolean isClose (MeshBase m1, MeshBase m2) {
      Point3d cen1 = new Point3d ();
      Point3d cen2 = new Point3d ();
      double r1 = m1.computeAverageRadius ();
      double r2 = m2.computeAverageRadius ();
      
      m1.computeCentroid (cen1);
      m2.computeCentroid (cen2);
      
      double dis = cen1.distance (cen2);
      double r = Math.min (r1, r2);
      double ratio = dis/r;
      
      if (ratio < 0.06) {
         return true;
      }
      
      return false;
   }

}
