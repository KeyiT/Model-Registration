package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.util.ArrayList;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.modelbase.RenderableComponentList;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;

public class EdgeAllocator implements InfoAllocator<EdgeInfo>{
   
   PolygonalMesh myMesh;
   FemModel3d myFem;
   
   HostType myHost = HostType.NULL;
   
   /**
    * Describe the type of edge host
    * @author KeyiTang
    */
   public enum HostType {
      NULL,
      POLYGONALMESH,
      FEMMODEL3D
   }

   public EdgeAllocator (PolygonalMesh mesh) {
      myMesh = mesh;
   }

   public EdgeAllocator (FemModel3d fem) {
      myFem = fem;
   }
   
   /**
    * set host type
    * @param type
    */
   public void setHostType (HostType type) {
      myHost = type;
   }
   
   
   @Override
   /**
    * make edge info list for host and allocate to 
    * <tt>reulstInfo</tt>
    */
   public void allocateInfo (CloudInfo<EdgeInfo> resultInfo) {
      if (myHost == HostType.NULL) {
         System.err.println ("Null Host Type");
      } 
      else if (myHost == HostType.POLYGONALMESH) {
         if (myMesh == null) {
            throw new ImproperStateException("Host not initialized");
         }
         makeEdgeInfo(resultInfo, myMesh);
      }
      else if (myHost == HostType.FEMMODEL3D) {
         if (myFem == null) {
            throw new ImproperStateException("Host not initialized");
         }
         makeEdgeInfo(resultInfo, myFem);
      }
   }
   
   public static void makeEdgeInfo (CloudInfo<EdgeInfo> rCI, PolygonalMesh mesh) {
      ArrayList<Face> faces = mesh.getFaces ();
      rCI.clearInfoList ();
      // clear all half-edge visited
      for (Face face : faces) {
         HalfEdge he = face.firstHalfEdge ();
         do {
            he.clearVisited();
            he = he.getNext ();
         } while (he != face.firstHalfEdge ());
      }
      // go through and actually count now
      for (Face face : faces) {
         HalfEdge he = face.firstHalfEdge ();
         do {
            if (!he.isVisited()) {
               he.setVisited();
               EdgeInfo info = new EdgeInfo (
                  he.head.getPosition (),
                  he.tail.getPosition ());
               rCI.addInfo (info);
               if (he.opposite != null) {
                  he.opposite.setVisited();
               }
            }
            he = he.getNext ();
         } while (he != face.firstHalfEdge ());
      }
   }
   
   public static void makeEdgeInfo (CloudInfo<EdgeInfo> rCI, FemModel3d fem) {
      RenderableComponentList<FemElement3d> eles = fem.getElements ();;
      rCI.clearInfoList ();
      for (FemElement3d ele : eles) {
         // for exclusion
         int [] edgeIndices = ele.getEdgeIndices ();
         int n = 2;
         for (int idxE = 0; idxE < edgeIndices.length; idxE += (n+1)) {
            n = edgeIndices[idxE];
            Point3d pnt1 = null;
            Point3d pnt2 = null;
            if (n == 2) {
               // avoid repeatly regularize on a same edge
               pnt1 = ele.getNodes ()[edgeIndices[idxE+1]].getPosition ();
               pnt2 = ele.getNodes ()[edgeIndices[idxE+2]].getPosition ();
            } 
            else if (n==3) {
               // avoid repeatly regularize on a same edge
               pnt1 = ele.getNodes ()[edgeIndices[idxE+1]].getPosition ();
               pnt2 = ele.getNodes ()[edgeIndices[idxE+3]].getPosition ();
            }
            if (pnt1 == null || pnt2 == null) {
               continue;
            }
            EdgeInfo info = new EdgeInfo (
               pnt1, pnt2);
            boolean flag = false;
            int idx = rCI.numInfos ();
            while(idx > 0) {
               idx--;
               ObjInfo tmp = rCI.getInfo (idx);
               if (((EdgeInfo)tmp).equal (info)) {
                  flag = true;
                  break;
               }
            }
            if (flag == false) {
               rCI.addInfo (info);
            }
         }
      }
   }

   @Override
   /**
    * make edge info list for host and added into
    * info-list of <tt>reulstInfo</tt>
    */
   public void addInfo (CloudInfo<EdgeInfo> resultInfo) {
      if (myHost == HostType.NULL) {
         System.err.println ("Null Host Type");
      } 
      else if (myHost == HostType.POLYGONALMESH) {
         if (myMesh == null) {
            throw new ImproperStateException("Host not initialized");
         }
         CloudInfo<EdgeInfo> infos = new CloudInfo<EdgeInfo> ();
         makeEdgeInfo(infos, myMesh);
         for (EdgeInfo info : infos.getInfos ()) {
            resultInfo.addInfo (info);
         }
      }
      else if (myHost == HostType.FEMMODEL3D) {
         if (myFem == null) {
            throw new ImproperStateException("Host not initialized");
         }
         CloudInfo<EdgeInfo> infos = new CloudInfo<EdgeInfo> ();
         makeEdgeInfo(infos, myFem);
         for (EdgeInfo info : infos.getInfos ()) {
            resultInfo.addInfo (info);
         }
      }
   }

}
