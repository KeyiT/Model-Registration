package artisynth.models.swallowingRegistrationTool.utilities;

import java.util.ArrayList;
import java.util.List;

import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.LineSegment;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;

public class BVTreeUtilities {

   /**
    * Returns a list of all <code>PolygonalMesh</code> face in <tt>mesh</tt> 
    * which inside a sphere.
    * 
    * @param faces returns all faces inside the sphere
    * @param center center of the sphere (world coordinates)
    * @param r sphere radius
    * @param mesh <code>PolygonalMesh</code> contains all faces to be checked;
    */
   public static void facesInsideSphere (
      List<Face> faces, Point3d center, double r, PolygonalMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            faces.add((Face)ele);
         }
      }
   }

   public static void verticesInsideSphere (
      List<Vertex3d> vtxs, Point3d center, double r, PointMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            vtxs.add((Vertex3d)ele);
         }
      }
   }

   public static void linesInsideShpere (
      List<LineSegment> lines, Point3d center, double r, PolylineMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            lines.add((LineSegment)ele);
         }
      }
   }

   /**
    * 
    * @param features
    * @param center
    * @param r
    * @param mesh
    */
   public static void featuresInsideSphere (
      List<Object> features, Point3d center, double r, MeshBase mesh) {
      if (mesh instanceof PolygonalMesh) {
         List<Face> faces = new ArrayList<Face>();
         facesInsideSphere(faces, center, r, (PolygonalMesh)mesh);
         features.addAll (faces);
      }
      else if (mesh instanceof PolylineMesh) {
         List<LineSegment> lines = new ArrayList<LineSegment>();
         linesInsideShpere(lines, center, r, (PolylineMesh)mesh);
         features.addAll (lines);
      }
      else if (mesh instanceof PointMesh) {
         List<Vertex3d> vtxs = new ArrayList<Vertex3d>();
         verticesInsideSphere(vtxs, center, r, (PointMesh)mesh);
         features.addAll (vtxs);
      }
   }

   /**
    * Returns a list of all leaf nodes in this tree which inside a sphere.
    * 
    * @param nodes returns all leaf nodes inside the sphere
    * @param center center of the sphere (world coordinates)
    * @param r sphere radius
    */
   public static void insideSphere (
      List<BVNode> nodes, Point3d center, double r, BVTree tree) {

      RigidTransform3d Xbvw = tree.getBvhToWorld ();
      if (Xbvw != RigidTransform3d.IDENTITY) {
         center = new Point3d (center);
         center.inverseTransform (Xbvw);
      }
      recursivelyInsideSphere (nodes, center, r, tree.getRoot());
   }

   private static void recursivelyInsideSphere (
      List<BVNode> nodes, Point3d origin, double r, BVNode node) {
      if (node.distanceToPoint (origin) < r) {
         if (node.isLeaf()) {
            nodes.add (node);
            return;
         }
         else {
            BVNode child = node.getFirstChild ();
            while (child != null) {
               recursivelyInsideSphere (nodes, origin, r, child);
               child = child.getNext ();
            }
         }
      }
   }

}
