package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.ArrayList;

import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;

public class VertexToPointCloudCorrespondence extends VertexToMeshCorrespondence{

   public VertexToPointCloudCorrespondence () {
      super();
   }

   // just place here for future usage
   /*
   @Override
   public ArrayList<Matrix3d> createSourceRelaxMatrices () {
      if (!srcRelaxEnabled) {
         return null;
      }
      int n = myCloudInfo.getInfos ().size ();
      ArrayList<Matrix3d> Covs = new ArrayList<Matrix3d>();

      for(int i = 0; i < n; i++) {
         if (!( myCloudInfo.getInfoFeature (i) instanceof VertexToMeshCorrespondence)) {
            throw new InvalidCorrespondenceException("Incompatible correspondence");
         }
         VertexToMeshCorrespondence feature = (VertexToMeshCorrespondence)
         myCloudInfo.getInfoFeature (i);
         Matrix3d Cov = null;
      
         Vertex3d self = feature.getSelf (Vertex3d.class);
         Vector3d nrm = new Vector3d ();
         
         if (self.equals (feature.myVertex)) {
            if (! isSourceToTargetEnabled ()) continue;
            if (!self.computeNormal (nrm)) {
               // TODO: handle this situation later
               System.err.println ("Source point unabled to be relaxed!");
               Covs.add (Cov);
               continue;
            }
            // TODO: XMeshTOWorld
         }
         else {
            if (!isTargetToSourceEnabled ()) continue;
            if (feature.getNearestFeature () instanceof Face) {
               Face face = (Face)feature.getNearestFeature ();
               face.computeNormal (nrm);
            }
            else if (feature.getNearestFeature () instanceof Vertex3d) {
               Vertex3d vtx = (Vertex3d)feature.getNearestFeature ();
               if (!vtx.computeNormal (nrm)) {
                  // TODO: handle this situation later
                  System.err.println ("Source point unabled to be relaxed!");
                  Covs.add (Cov);
                  continue;
               }
            }
         }
         
         Cov = new Matrix3d ();
         Matrix3d R = new Matrix3d ();
         nrm.normalize ();
         
         Vector3d vec0 = new Vector3d ();
         int idxMax = nrm.maxAbsIndex ();
         int idxMin = nrm.minAbsIndex ();
         int idxMid = 3 - idxMax -idxMin;
         vec0.set (idxMax, 1.0 / nrm.get (idxMax));
         if (Math.abs (nrm.get (idxMid)) > 1E-16) {
            vec0.set (idxMid, - 1.0/nrm.get (idxMid));
         }
         vec0.normalize ();
         
         Vector3d vec1 = new Vector3d ();
         vec1.cross (nrm, vec0);
         
         R.setColumn (0, nrm);
         R.setColumn (1, vec0);
         R.setColumn (2, vec1);
         
         nrm.scale (1.0/srcCov);
         Cov.setColumn (0, nrm);
         Cov.setColumn (1, vec0);
         Cov.setColumn (2, vec1);
         Cov.mulTranspose (R);
         
         Covs.add (Cov);
      }
      return Covs;
   }
   
   @Override
   public ArrayList<Matrix3d> createTargetRelaxMatrices () {
      
      if (!tgtRelaxEnabled) {
         return null;
      }
      int n = myCloudInfo.getInfos ().size ();
      ArrayList<Matrix3d> Covs = new ArrayList<Matrix3d> ();

      for(int i = 0; i < n; i++) {
         if (!( myCloudInfo.getInfoFeature (i) instanceof VertexToMeshCorrespondence)) {
            throw new InvalidCorrespondenceException("Incompatible correspondence");
         }
         VertexToMeshCorrespondence feature = (VertexToMeshCorrespondence)
         myCloudInfo.getInfoFeature (i);
         Matrix3d Cov = null;
      
         Vertex3d partner = feature.getPartner (Vertex3d.class);
         Vector3d nrm = new Vector3d ();
         
         if (partner.equals (feature.myVertex)) {
            if (!isTargetToSourceEnabled ()) continue;
            if (!partner.computeNormal (nrm)) {
               // TODO: handle this situation later
               System.err.println ("Target point unabled to be relaxed!");
               Covs.add (Cov);
               continue;
            }
            
            if (partner.getMesh () != null && 
            !partner.getMesh ().meshToWorldIsIdentity ()) {
               nrm.transform (partner.getMesh ().getMeshToWorld ());
            }
         }
         else {
            if (! isSourceToTargetEnabled ()) continue;
            if (feature.getNearestFeature () instanceof Face) {
               Face face = (Face)feature.getNearestFeature ();
               face.clearNormal ();
               nrm = face.getWorldNormal ();
            }
            else if (feature.getNearestFeature () instanceof Vertex3d) {
               Vertex3d vtx = (Vertex3d)feature.getNearestFeature ();
               if (!vtx.computeNormal (nrm)) {
                  // TODO: handle this situation later
                  System.err.println ("Target point unabled to be relaxed!");
                  Covs.add (Cov);
                  continue;
               }
               if (vtx.getMesh () != null && 
               !vtx.getMesh ().meshToWorldIsIdentity ()) {
                  nrm.transform (vtx.getMesh ().getMeshToWorld ());
               }
            }
         }
         
         
         Cov = new Matrix3d ();
         Matrix3d R = new Matrix3d ();
         nrm.normalize ();
         
         Vector3d vec0 = new Vector3d ();
         int idxMax = nrm.maxAbsIndex ();
         int idxMin = nrm.minAbsIndex ();
         int idxMid = 3 - idxMax -idxMin;
         vec0.set (idxMax, 1.0 / nrm.get (idxMax));
         if (Math.abs (nrm.get (idxMid)) > 1E-16) {
            vec0.set (idxMid, - 1.0/nrm.get (idxMid));
         }
         vec0.normalize ();
         
         Vector3d vec1 = new Vector3d ();
         vec1.cross (nrm, vec0);
         
         R.setColumn (0, nrm);
         R.setColumn (1, vec0);
         R.setColumn (2, vec1);
         
         nrm.scale (1.0/tgtCov);
         Cov.setColumn (0, nrm);
         Cov.setColumn (1, vec0);
         Cov.setColumn (2, vec1);
         Cov.mulTranspose (R);
         
         Covs.add (Cov);
      }
      return Covs;

   }*/
}
