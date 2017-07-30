package artisynth.models.swallowingRegistrationTool.transformers;

import artisynth.models.swallowingRegistrationTool.utilities.*;
import maspack.geometry.AABBTree;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;

public class NFFD3dControlUnit implements Boundable{

   // lower bounds of knots
   protected int [][] mySpanRange = new int [3][2];
   int [] myVertexIndices = new int [3];
   Vector4d [] myVertices = new Vector4d [8];
   NURBSVolume myNURBS;
   
   MeshConvexHull myCH;
   
   public NFFD3dControlUnit () {
      // TODO Auto-generated constructor stub
   }
   
   public NFFD3dControlUnit (NURBSVolume nurbs) {
      myNURBS = nurbs;
   }
   
   public void setNURBS (NURBSVolume nurbs) {
      myNURBS = nurbs;
   }
   
   
   public void setCtrlVertices (int [] indices) {
      if (indices.length != 3) {
         throw new IllegalArgumentException ("Indices dimension must be 3!");
      }
      
      if (myNURBS == null) {
         throw new ImproperStateException ("NURBS-Volume not initialized!");
      }
      
      int [] degrees = myNURBS.getDegrees ();
      for (int i = 0; i < 3; i++) {
         myVertexIndices [i] = indices[i];
         mySpanRange[i][0] = indices[i] + 1;
         mySpanRange[i][1] = indices[i] + degrees[i];
      }
      
      int idx = 0;
      for (int i = 0; i < 2; i++) {
         for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
               Vector4d pnt = myNURBS.getControlPoint (
                  myNURBS.indexMap (indices[0]+i, indices[1]+j, indices[2]+k));
               myVertices [idx++] = pnt;
            }
         }
      }
   }
   
   public Vector4d[] getCtrlVertices () {
       Vector4d [] vsr = new Vector4d [8];
       for (int i = 0; i < 8; i++) {
          vsr[i] = myVertices[i];
       }
       return vsr;
   }
   
   public int [][] getKnotSpanRange () {
      int [][] range = mySpanRange.clone ();
      return range;
   }
   
   /**
    * 
    * @return the indices of control points that can
    * influence points in this unit
    */
   public int[] getSupportRange () {
      int [] min = myVertexIndices;
      int [] max = myVertexIndices.clone ();
      
      int [] degrees = myNURBS.getDegrees ();
      for (int i = 0; i < 3; i++) {
         max[i] += degrees [i];
      }
      // TODO
      return null;
   }
   

   @Override
   public int numPoints () {
      return 8;
   }

   @Override
   public Point3d getPoint (int idx) {
      Point3d pnt  = new Point3d ();
      Vector4d vtx = myVertices[idx];
      pnt.x = vtx.x;
      pnt.y = vtx.y;
      pnt.z = vtx.z;
      return pnt;
   }

   @Override
   public void computeCentroid (Vector3d centroid) {
      centroid.setZero ();
      for (int i = 0; i < 8; i++) {
         Vector4d vtx = myVertices[i];
         
         centroid.x += vtx.x;
         centroid.y += vtx.y;
         centroid.z += vtx.z;
      }
      centroid.scale (0.125);
   }

   @Override
   public void updateBounds (Vector3d min, Vector3d max) {
      for (int i = 0; i < 8; i++) {
         Point3d pnt = getPoint(i);
         pnt.updateBounds (min, max);
      }
   }
   
   public void generateConvexHull () {
      Point3d [] pnts = new Point3d [8];
      for (int i = 0; i < 8; i++) {
         pnts [i] = getPoint(i);
      }
      myCH = new MeshConvexHull (pnts);
   }
   
   public boolean isInsideConvexHull (Point3d pntInWorld) {
      if (myCH == null) {
         throw new ImproperStateException ("Convex hull not initialized!");
      }
      return myCH.isInsideConvexHull (pntInWorld);
   }

   @Override
   public double computeCovariance (Matrix3d C) {
      return -1;
   }


}
