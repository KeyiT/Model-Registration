package artisynth.models.swallowingRegistrationTool.utilities;

import java.io.IOException;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;

import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.subjectFrank.SubjectModel;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.ScaledRigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.ArraySort;
import maspack.util.InternalErrorException;

/**
 * 
 * @author KeyiTang
 *
 */
public class GeometryOperator {

   public enum TriangleType {
      Obtuse,
      Right,
      Acute,
      Unknown
   }

   /**
    * detect triangle type
    * @param face face to be detected; if face is not a 
    * triangle, return {@link TriangleType#Unknown};
    * @param tol tolerance for right triangle
    * @return triangle type
    */
   public static TriangleType getTriangleType (Face face, double tol) {
      if (! face.isTriangle ()) {
         return TriangleType.Unknown;
      }

      // get squared edge length
      double [] lens = new double [3];
      for (int i = 0; i < 3; i++) {
         double len2 = face.getEdge (i).lengthSquared ();
         lens [i] = len2;
      }

      ArraySort.quickSort (lens);

      double a2b2 = lens [0] + lens [1];
      double c2 = lens [2];


      if (Math.sqrt (Math.abs (a2b2 - c2)) <= tol) {
         return TriangleType.Right;
      }
      else if (a2b2 < c2) {
         return TriangleType.Obtuse;
      }
      else {
         return TriangleType.Acute;
      }
   }

   public static LinkedHashSet<Face> getNeighborFaces (Vertex3d vtx) {
      LinkedHashSet<Face> neighbors = new LinkedHashSet<Face> ();
      Iterator<HalfEdge> egs = vtx.getIncidentHalfEdges ();

      while (egs.hasNext ()) {
         HalfEdge edge = egs.next ();
         neighbors.add (edge.getFace ());
      }

      return neighbors;
   }

   public static LinkedHashSet<Vertex3d> getNeighborVetices (Vertex3d vtx) {
      LinkedHashSet<Vertex3d> neighbors = new LinkedHashSet <Vertex3d> ();
      Iterator<HalfEdge> egs = vtx.getIncidentHalfEdges ();

      while (egs.hasNext ()) {
         HalfEdge edge = egs.next ();
         neighbors.add (edge.getTail ());
         if (edge.getNext ().opposite == null) {
            // for boundary edge
            neighbors.add (edge.getHead ());
         }
      }

      return neighbors;
   }

   /**
    * catenate vertex-world-positions in a vector
    * @param mesh
    * @param removeIsolaters ignore isolated vertices
    * @return
    */
   public static VectorNd assembleVerticesVector (
      PolygonalMesh mesh, boolean removeIsolaters) {

      int numV = mesh.numVertices ();
      if (removeIsolaters) {
         numV -= mesh.numDisconnectedVertices ();
      }

      VectorNd vts = new VectorNd (numV * 3);

      for (int i = 0; i < mesh.numVertices (); i++) {
         vts.setSubVector (i*3, mesh.getVertex (i).getWorldPoint ());
      }

      return vts;
   }

   /**
    * Compute mean curvature normal of a triangular 2-manifold mesh
    * @param rLs if not null, return Laplacian operator
    * @param rMass if not null, return lumped Voronoi mass matrix
    * @param mesh triangular 2-manifold mesh
    * @param linearlyPrecise has linear precision for both open and close 
    * mesh
    * @return if <tt>mesh</tt> is not triangular 2-manifold mesh return 
    * null, otherwise return mean curvature normal vector
    */
   public static VectorNd computeMeanCurvatureNormal (
      SparseBlockMatrix rLs, SparseBlockMatrix rMass,
      PolygonalMesh mesh, boolean linearlyPrecise) {

      SparseBlockMatrix Ls = createLaplacianMapMatrix (mesh, linearlyPrecise);
      if (Ls == null) {
         return null;
      }

      SparseBlockMatrix Mass = createVoronoiMassMatrix (mesh);
      if (rMass != null) {
         rMass.set (Mass);
      }

      invertMassMatrix (Mass);

      VectorNd vts = assembleVerticesVector (mesh, true);
      VectorNd tmp = new VectorNd ();
      VectorNd K_H = new VectorNd ();
      Ls.mul (tmp, vts);
      Mass.mul (K_H, tmp);

      if (rLs != null) {
         rLs.set (Ls);
      }


      return K_H;
   }

   /**
    * 
    * @param mesh input triangular manifold mesh
    * @return lumped diangonal Voronoi mass matrix
    */
   public static SparseBlockMatrix createVoronoiMassMatrix (
      PolygonalMesh mesh) {
      if (! mesh.isManifold ()) {
         System.err.println ("Not manifold!");
         return null;
      }

      if (! mesh.isTriangular ()) {
         System.err.println ("Not triangular mesh!");
         return null;
      }

      int numVertices = mesh.numVertices ();
      numVertices -= mesh.numDisconnectedVertices ();

      int [] nums = new int [numVertices];
      for (int i = 0; i < numVertices; i++) {
         nums [i] = 3;
      }

      SparseBlockMatrix Mass = new SparseBlockMatrix (nums, nums);
      for (int i = 0; i < mesh.numVertices (); i++) {
         double A_ii = computeVoronoiArea (mesh.getVertex (i));
         addMapBlock (Mass, A_ii, i, i);
      }

      return Mass;
   }

   /**
    * create cotangent Laplacian Matrix (Laplace-Beltrami operator)
    * @param mesh input mesh, triangular manifold mesh;
    * @param linearPrecision 
    */
   public static SparseBlockMatrix createLaplacianMapMatrix (
      PolygonalMesh mesh, boolean linearPrecision) {

      if (! mesh.isManifold ()) {
         System.err.println ("Not manifold!");
         return null;
      }

      if (! mesh.isTriangular ()) {
         System.err.println ("Not triangular mesh!");
         return null;
      }

      int numVertices = mesh.numVertices ();
      numVertices -= mesh.numDisconnectedVertices ();

      int [] nums = new int [numVertices];
      for (int i = 0; i < numVertices; i++) {
         nums [i] = 3;
      }

      SparseBlockMatrix Map = new SparseBlockMatrix (nums, nums);
      for (int i = 0; i < mesh.numVertices (); i++) {
         if (mesh.getVertex (i).firstIncidentHalfEdge () == null) {
            // ignore isolated vertex
            continue;
         }
         addLaplacianMap (Map, mesh, i, linearPrecision);
      }

      return Map;
   }

   /**
    * create cotangent Laplacian Matrix (Laplace-Beltrami operator) for 
    * a single cell
    * @param mesh input mesh, triangular manifold mesh;
    * @param vtx the center vertex of the interest cell
    * @param linearPrecision 
    */
   public static SparseBlockMatrix createLaplacianMapMatrix (
      PolygonalMesh mesh, Vertex3d vtx, boolean linearPrecision) {

      if (! mesh.isManifold ()) {
         System.err.println ("Not manifold!");
         return null;
      }

      if (! mesh.isTriangular ()) {
         System.err.println ("Not triangular mesh!");
         return null;
      }

      if (! mesh.containsVertex (vtx)) {
         System.err.println ("vertex not belong to "
         + "the input mesh!");
         return null;
      }

      int numVertices = mesh.numVertices ();
      numVertices -= mesh.numDisconnectedVertices ();

      int [] nums = new int [numVertices];
      for (int i = 0; i < numVertices; i++) {
         nums [i] = 3;
      }
      int [] numr = {3};

      SparseBlockMatrix Map = new SparseBlockMatrix (numr, nums);

      if (vtx.firstIncidentHalfEdge () == null) {
         // ignore isolated vertex
         System.err.println ("warning: isolated vertex!");
         return null;
      }
      addLaplacianMap (Map, mesh, vtx, 0, linearPrecision);

      return Map;
   }

   /**
    * 
    * @param Map
    * @param mesh
    * @param iVer
    * @param linearPrecision
    */
   public static void addLaplacianMap (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      int iVer, boolean linearPrecision) {

      addLaplacianMap (Map, mesh, mesh.getVertex (iVer), 
         iVer, linearPrecision);
   }

   /**
    * 
    * @param Map
    * @param mesh
    */
   public static void addLaplacianMap (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      Vertex3d vtx, int iVer, boolean linearPrecision) {

      Iterator<HalfEdge> hit = vtx.getIncidentHalfEdges();
      HalfEdge [] bhedges = new HalfEdge [2];

      // loop through 1-ring neighbor
      while (hit.hasNext()) {
         HalfEdge he = hit.next();

         if (he.opposite == null) {
            // boundary edge
            if (bhedges[0] != null) {
               throw new InternalErrorException (
               "not manifold mesh");
            }
            bhedges[0] = he;
            addBoundaryLaplacianMap (Map, mesh, he, vtx, 
               iVer, he.getTail ().getIndex ());

            if (he.getNext ().opposite == null) {
               // boundary edge
               if (bhedges[1] != null) {
                  throw new InternalErrorException (
                  "not manifold mesh");
               }
               bhedges[1] = he.getNext ();
               addBoundaryLaplacianMap (Map, mesh, he.getNext (), vtx, 
                  iVer, he.getNext ().getHead ().getIndex ());
            }
            continue;
         }

         addInteriorLaplacianMap (Map, mesh, he, 
            iVer, he.getTail ().getIndex ());

         if (he.getNext ().opposite == null) {
            // boundary edge
            if (bhedges[1] != null) {
               throw new InternalErrorException (
               "not manifold mesh");
            }
            bhedges[1] = he.getNext ();
            addBoundaryLaplacianMap (Map, mesh, he.getNext (), vtx, 
               iVer, he.getNext ().getHead ().getIndex ());
         }
      }

      if (linearPrecision) {
         if (bhedges [0] != null) {
            if (bhedges [1] == null) {
               throw new InternalErrorException (
               "not manifold mesh");
            }
            addBoundaryNormalDerivative (
               Map, mesh, bhedges[0], vtx, iVer, bhedges[0].getTail ().getIndex ());
            addBoundaryNormalDerivative (
               Map, mesh, bhedges[1], vtx, iVer, bhedges[1].getHead ().getIndex ());
         }
      }
   }

   /**
    * Assuming <tt>mesh</tt> is a triangular mesh, and is 
    * manifold. The incident <tt>edge</tt> edge_ij is a 
    * interior edge(not on border). 
    * 
    * @param Map
    * @param mesh
    * @param edge the incident edge_ij
    * @return w_ij = ( cot(alph_ij) + cot(beta_ij) ) / 2
    */
   public static double addInteriorLaplacianMap (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      HalfEdge edge, int iVer, int jVer) {

      if (edge == null) {
         throw new InternalErrorException ("");
      }
      if (edge.opposite == null) {
         throw new InternalErrorException ("");
      }

      Face face0 = edge.getFace ();
      Face face1 = edge.opposite.getFace ();

      double cot0 = cotIJ (face0, edge);
      double cot1 = cotIJ (face1, edge);

      double w_ij = (cot0 + cot1) / 2.0;

      if (Map != null) {
         addMapBlock (Map, w_ij, iVer, jVer);
         addMapBlock (Map, -w_ij, iVer, edge.getHead ().getIndex ());
      }

      return w_ij;
   }

   /**
    * Assuming <tt>mesh</tt> is a triangular mesh, and is 
    * manifold mesh. The incident <tt>edge</tt> edge_ij is a 
    * boundary edge.
    * @param Map
    * @param mesh
    * @param edge the incident edge_ij
    * @return w_ij = cot(alph_ij) / 2
    */
   public static double addBoundaryLaplacianMap (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      HalfEdge edge, Vertex3d vtxI, int iVer, int jVer) {

      if (edge == null) {
         throw new InternalErrorException ("");
      }
      if (edge.opposite != null) {
         throw new InternalErrorException ("");
      }

      Face face0;
      face0 = edge.getFace ();

      double cot0 = cotIJ (face0, edge);

      double w_ij = cot0 / 2.0;

      if (Map != null) {
         addMapBlock (Map, w_ij, iVer, jVer);
         addMapBlock (Map, -w_ij, iVer, vtxI.getIndex ());
      }

      return w_ij;
   }

   /**
    * Assuming <tt>mesh</tt> is a triangular mesh, and is 
    * manifold mesh. The incident <tt>edge</tt> edge_ij is a 
    * boundary edge.
    * <p>
    * Reference: <i>Linear Subspace Design for Real-Time Shape Deformation; 
    * Yu Wang, et. al.</i>
    * @param Map
    * @param mesh
    * @param edge the incident edge_ij
    * @param iVer assuming iVer is the index of the center vertex
    * @param jVer
    * @return 
    * <p>
    * n_ki = cot(gamma_ki) 
    * <p>
    * n_kj = cot(gamma_kj)
    */
   public static double addBoundaryNormalDerivative (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      HalfEdge edge, int iVer, int jVer) {

      return addBoundaryNormalDerivative (Map, mesh, edge, 
         mesh.getVertex (iVer), iVer, jVer);
   }

   /**
    * Assuming <tt>mesh</tt> is a triangular mesh, and is 
    * manifold mesh. The incident <tt>edge</tt> edge_ij is a 
    * boundary edge.
    * <p>
    * Reference: <i>Linear Subspace Design for Real-Time Shape Deformation; 
    * Yu Wang, et. al.</i>
    * @param Map
    * @param mesh
    * @param edge the incident edge_ij
    * @param vtI center vertex of this cell
    * @param iVer 
    * @param jVer
    * @return 
    * <p>
    * n_ki = cot(gamma_ki) 
    * <p>
    * n_kj = cot(gamma_kj)
    */
   public static double addBoundaryNormalDerivative (
      SparseBlockMatrix Map, PolygonalMesh mesh, 
      HalfEdge edge, Vertex3d vtxI, int iVer, int jVer) {

      if (edge == null) {
         throw new InternalErrorException ("");
      }
      if (edge.opposite != null) {
         throw new InternalErrorException ("");
      }

      Vertex3d vtxJ = null;
      if (edge.getHead () == vtxI) {
         vtxJ = edge.getTail ();
      }
      else if (edge.getTail () == vtxI){
         vtxJ = edge.getHead ();
      }
      else {
         throw new InternalErrorException ("");
      }
      Vertex3d vtxK = null;
      Face face = edge.getFace ();

      Vertex3d [] tmp = face.getTriVertices ();
      for (int i = 0; i < 3; i++) {
         if (tmp [i] != vtxI && tmp [i] != vtxJ)  {
            vtxK = tmp [i];
         }
      }

      if (vtxK == null) {
         throw new InternalErrorException ("");
      }

      double cotIK = cotIK (face, vtxJ);
      double cotJK = cotIK (face, vtxI);

      double n_ki = cotIK / 2.0;
      double n_kj = cotJK / 2.0;


      addMapBlock (Map, -(n_ki + n_kj), iVer, vtxK.getIndex ());
      addMapBlock (Map, n_kj, iVer, jVer);
      addMapBlock (Map, n_ki, iVer, vtxI.getIndex ());

      return (n_ki + n_kj);
   }

   /**
    * 
    * @param face triagular face
    * @param vtxIK vertex ik
    * @return cotangent value of the angle_ik at vertex <tt>vtxIK</tt>;
    */
   protected static double cotIK (Face face, Vertex3d vtxIK) {

      Vertex3d [] tmp = face.getTriVertices ();
      Vertex3d [] vts = new Vertex3d [2];
      int idx = 0;
      for (int i = 0; i < 3; i++) {
         if (tmp [i] != vtxIK) {
            if (idx == 2) {
               throw new InternalErrorException ("");
            }
            vts [idx++] = tmp [i];
         }
      }

      Vector3d e0 = new Vector3d (vts [0].getPosition ());
      e0.sub (vtxIK.getPosition ());

      Vector3d e1 = new Vector3d (vts [1].getPosition ());
      e1.sub (vtxIK.getPosition ());

      double cot0 = cotangent (e0, e1);

      return cot0;
   }

   /**
    * 
    * @param face triangular face
    * @param edge edge_ij opposite to vertex vertex_ij
    * @param vtx vertex i
    * @return cotangent value of the angle_ij opposite to <tt>edge</tt>;
    * 
    */
   protected static double cotIJ (Face face, HalfEdge edge) {

      Vertex3d [] tmp = face.getTriVertices ();
      Vertex3d vtxIJ = null;
      for (Vertex3d v :  tmp) {
         if ((v != edge.getHead ()) && (v != edge.getTail ())) {
            vtxIJ = v;
            break;
         }
      }

      if (vtxIJ == null) {
         throw new InternalErrorException (
         "Unknown vertex ij");
      }


      Vector3d e0 = new Vector3d (edge.getHead ().getPosition ());
      e0.sub (vtxIJ.getPosition ());

      Vector3d e1 = new Vector3d (edge.getTail ().getPosition ());
      e1.sub (vtxIJ.getPosition ());

      double cot0 = cotangent (e0, e1);

      return cot0;
   }


   /**
    * 
    * @param vtx vertex
    * @return 1-ring Voronoi region area around vertex <tt>vtx</tt>
    */
   public static double computeVoronoiArea (Vertex3d vtx) {
      LinkedHashSet <Face> faces = getNeighborFaces (vtx);
      double Amix = 0;

      for (Face face : faces) {
         try {
            Amix += computeVoronoiArea (vtx, face);
         }
         catch (ImproperStateException e) {
            System.err.println ("not pure triangular mesh!");
            return 0;
         }

      }

      return Amix;
   }

   /**
    * 
    * @param vtx vertex
    * @param face triangle face
    * @return Voronoi region area of the triangle face
    */
   protected static double computeVoronoiArea (Vertex3d vtx, Face face) 
   throws ImproperStateException{
      double Amix = 0;
      TriangleType type = getTriangleType (face, 0);

      if (type == TriangleType.Unknown) {
         throw new ImproperStateException (
         "Not triangle");
      }
      else if (type == TriangleType.Acute) {

         Vertex3d [] tmp = face.getTriVertices ();
         Vertex3d [] vts = new Vertex3d [2];
         int idx = 0;
         for (int i = 0; i < 3; i++) {
            if (tmp [i] != vtx)  {
               vts [idx++] = tmp [i];
            }
         }

         Vector3d e0 = new Vector3d (vts [0].getPosition ());
         e0.sub (vts [1].getPosition ());

         Vector3d e1 = new Vector3d (vtx.getPosition ());
         e1.sub (vts [0].getPosition ());

         Vector3d e2 = new Vector3d (vtx.getPosition ());
         e2.sub (vts [1].getPosition ());

         double cot1 = cotangent (e0, e2);
         e0.negate ();
         double cot0 = cotangent (e0, e1);

         double el2 = e2.normSquared ();
         double el1 = e1.normSquared ();

         double s0 = el2 * cot0;
         double s1 = el1 * cot1;

         Amix = (s0 + s1) / 8.0;
      }
      else {

         double s = FEMQualityUtilities.triangleArea (
            face.getVertex (0).getPosition (), 
            face.getVertex (1).getPosition (), 
            face.getVertex (2).getPosition ());

         Vertex3d [] tmp = face.getTriVertices ();
         Vertex3d [] vts = new Vertex3d [2];
         int idx = 0;
         for (int i = 0; i < 3; i++) {
            if (tmp [i] != vtx)  {
               vts [idx++] = tmp [i];
            }
         }

         Vector3d e0 = new Vector3d (vts [0].getPosition ());
         e0.sub (vts [1].getPosition ());

         Vector3d e1 = new Vector3d (vtx.getPosition ());
         e1.sub (vts [0].getPosition ());

         Vector3d e2 = new Vector3d (vtx.getPosition ());
         e2.sub (vts [1].getPosition ());

         double el2 = e2.normSquared ();
         double el1 = e1.normSquared ();
         double el0 = e0.normSquared ();

         if (el0 > (el1 + el2)) {
            // the angle at vtx is obtuse
            Amix = s / 2.0;
         }
         else {
            Amix = s / 4.0;
         }
      }

      return Amix;
   }

   /**
    * Find local rotation for cell with center-vertex <tt>vtxI</tt>;
    * <p>
    * Reference : <i> As-Rigid-As-Possible Surface Modeling, 
    * Sorkine O. et. al. 2007</i>
    * @param vtxI center vertex 
    * @param UndeformedPositions map from current position to undeformed 
    * position
    * @param CellMap Laplacian map matrix of the containing mesh
    * @return local rotation matrix
    */
   public static ScaledRigidTransform3d computeCellLocalConformalMap (
      Vertex3d vtxI, Map <Point3d, Point3d> UndeformedPositions, 
      SparseBlockMatrix CellMap) {

      Iterator<HalfEdge> hit = vtxI.getIncidentHalfEdges();
      HalfEdge bhedges = null;

      Point3d pntI = new Point3d ();
      vtxI.getWorldPoint (pntI);

      Point3d oldPntI = new Point3d ();
      oldPntI.set (UndeformedPositions.
         get (vtxI.getPosition ()));

      VectorNd wijs = new VectorNd ();

      MatrixNd Eijs = new MatrixNd (0, 3);
      MatrixNd oldEijs = new MatrixNd (0, 3);

      // loop through 1-ring neighbor
      int idx = 0;
      double w_ij = 0;
      HalfEdge he;
      boolean validWeights = true;
      while (hit.hasNext()) {
         he = hit.next();
         w_ij = GeometryOperator.makeEdgeVector (
            Eijs, oldEijs, vtxI, pntI, oldPntI, he, 
            UndeformedPositions, CellMap);
         if (w_ij < 0) {
            validWeights = false;
         }
         wijs.setSize (idx+1);
         wijs.set (idx, w_ij);
         idx ++;

         if (he.getNext ().opposite == null) {
            he = he.getNext ();
            w_ij = GeometryOperator.makeEdgeVector (
               Eijs, oldEijs, vtxI, pntI, oldPntI, he, 
               UndeformedPositions, CellMap);
            if (w_ij < 0) {
               validWeights = false;
            }
            wijs.setSize (idx+1);
            wijs.set (idx, w_ij);
            idx ++;
         }
      }

      // scaling
      double snew = 0;
      for (int i = 0; i < idx; i++) {
         for (int j = 0; j < 3; j++) {
            snew += Eijs.get (i, j) * Eijs.get (i, j) * wijs.get (i);
         }
      }

      double sold = 0;
      for (int i = 0; i < idx; i++) {
         for (int j = 0; j < 3; j++) {
            sold += oldEijs.get (i, j) * oldEijs.get (i, j)  * wijs.get (i);
         }
      }

      double s = Math.sqrt (snew / sold);

      // cotangent weight violates the positive weights
      // property of a desired Laplacian; obtuse angles cause
      // det(VU^T) < 0;
      // solution: avoid it by assigning uniform weights
      // Note: a modified ARAP does no have this issue !
      if (validWeights) {
         for (int i = 0; i < idx; i++) {
            for (int j = 0; j < 3; j++) {
               double val = Eijs.get (i, j);
               Eijs.set (i, j, val * wijs.get (i));
            }
         }
      }

      // for test
      //System.out.println ("weight valid: " + validWeights);

      Matrix3d Cov = new Matrix3d ();
      MatrixNd Tmp = new MatrixNd ();
      Tmp.mulTransposeLeft (oldEijs, Eijs);
      Cov.set (Tmp);

      Matrix3d U = new Matrix3d();
      Matrix3d V = new Matrix3d();
      SVDecomposition svd = new SVDecomposition (Cov);
      U.set (svd.getU ());
      V.set (svd.getV ());
      double detU = U.orthogonalDeterminant();
      double detV = V.orthogonalDeterminant();

      if (detV * detU < 0) { /* then one is negative and the other positive */
         if (detV < 0) { /* negative last column of V */
            V.m02 = -V.m02;
            V.m12 = -V.m12;
            V.m22 = -V.m22;
         }
         else /* detU < 0 */
         { /* negative last column of U */
            U.m02 = -U.m02;
            U.m12 = -U.m12;
            U.m22 = -U.m22;
         }
      }

      // rotation
      RotationMatrix3d BestR = new RotationMatrix3d ();
      Matrix3d VUT = new Matrix3d ();
      VUT.mulTransposeRight (V, U);
      BestR.set (VUT);

      ScaledRigidTransform3d map = new ScaledRigidTransform3d ();
      map.setRotation (BestR);
      map.setScale (s);

      return map;
   }

   /**
    * Find local rotation for cell with center-vertex <tt>vtxI</tt>;
    * <p>
    * Reference : <i> As-Rigid-As-Possible Surface Modeling, 
    * Sorkine O. et. al. 2007</i>
    * @param vtxI center vertex 
    * @param UndeformedPositions map from current position to undeformed 
    * position
    * @param CellMap Laplacian map matrix of the containing mesh
    * @return local rotation matrix
    */
   public static RotationMatrix3d computeCellLocalRotation (
      Vertex3d vtxI, Map <Point3d, Point3d> UndeformedPositions, 
      SparseBlockMatrix CellMap) {

      Iterator<HalfEdge> hit = vtxI.getIncidentHalfEdges();
      HalfEdge bhedges = null;

      Point3d pntI = new Point3d ();
      vtxI.getWorldPoint (pntI);

      Point3d oldPntI = new Point3d ();
      oldPntI.set (UndeformedPositions.
         get (vtxI.getPosition ()));

      VectorNd wijs = new VectorNd ();

      MatrixNd Eijs = new MatrixNd (0, 3);
      MatrixNd oldEijs = new MatrixNd (0, 3);

      // loop through 1-ring neighbor
      int idx = 0;
      double w_ij = 0;
      HalfEdge he;
      boolean validWeights = true;
      while (hit.hasNext()) {
         he = hit.next();
         w_ij = GeometryOperator.makeEdgeVector (
            Eijs, oldEijs, vtxI, pntI, oldPntI, he, 
            UndeformedPositions, CellMap);
         if (w_ij < 0) {
            validWeights = false;
         }
         wijs.setSize (idx+1);
         wijs.set (idx, w_ij);
         idx ++;

         if (he.getNext ().opposite == null) {
            he = he.getNext ();
            w_ij = GeometryOperator.makeEdgeVector (
               Eijs, oldEijs, vtxI, pntI, oldPntI, he, 
               UndeformedPositions, CellMap);
            if (w_ij < 0) {
               validWeights = false;
            }
            wijs.setSize (idx+1);
            wijs.set (idx, w_ij);
            idx ++;
         }
      }

      // cotangent weight violates the positive weights
      // property of a desired Laplacian; obtuse angles cause
      // det(VU^T) < 0;
      // solution: avoid it by assigning uniform weights
      // Note: a modified ARAP does no have this issue !
      if (validWeights) {
         for (int i = 0; i < idx; i++) {
            for (int j = 0; j < 3; j++) {
               double val = Eijs.get (i, j);
               Eijs.set (i, j, val * wijs.get (i));
            }
         }
      }

      // for test
      //System.out.println ("weight valid: " + validWeights);

      Matrix3d Cov = new Matrix3d ();
      MatrixNd Tmp = new MatrixNd ();
      Tmp.mulTransposeLeft (oldEijs, Eijs);
      Cov.set (Tmp);

      Matrix3d U = new Matrix3d();
      Matrix3d V = new Matrix3d();
      SVDecomposition svd = new SVDecomposition (Cov);
      U.set (svd.getU ());
      V.set (svd.getV ());
      double detU = U.orthogonalDeterminant();
      double detV = V.orthogonalDeterminant();

      if (detV * detU < 0) { /* then one is negative and the other positive */
         if (detV < 0) { /* negative last column of V */
            V.m02 = -V.m02;
            V.m12 = -V.m12;
            V.m22 = -V.m22;
         }
         else /* detU < 0 */
         { /* negative last column of U */
            U.m02 = -U.m02;
            U.m12 = -U.m12;
            U.m22 = -U.m22;
         }
      }

      RotationMatrix3d BestR = new RotationMatrix3d ();
      Matrix3d VUT = new Matrix3d ();
      VUT.mulTransposeRight (V, U);
      BestR.set (VUT);

      return BestR;
   }

   private static double makeEdgeVector (MatrixNd Eijs, MatrixNd oldEijs, 
      Vertex3d vtxI, Point3d pntI, Point3d oldPntI, HalfEdge he, 
      Map <Point3d, Point3d> UndeformedPositions, 
      SparseBlockMatrix CellMap) {

      Vertex3d vtxJ;
      if (he.getHead () == vtxI) {
         vtxJ = he.getTail ();
      }
      else if (he.getTail () == vtxI) {
         vtxJ = he.getHead ();
      }
      else {
         throw new InternalErrorException ("");
      }


      Point3d pntJ = new Point3d ();
      vtxJ.getWorldPoint (pntJ);

      Point3d oldpntJ = new Point3d ();
      oldpntJ.set (UndeformedPositions.
         get (vtxJ.getPosition ()));

      Vector3d Eij = new Vector3d ();
      Eij.sub (pntI, pntJ);
      Vector3d oldEij = new Vector3d ();
      oldEij.sub (oldPntI, oldpntJ);

      Eijs.setSize (Eijs.rowSize ()+1, 3);
      oldEijs.setSize (oldEijs.rowSize ()+1, 3);
      Eijs.setRow (Eijs.rowSize ()-1, Eij);
      oldEijs.setRow (oldEijs.rowSize ()-1, oldEij);

      MatrixBlock blk = CellMap.getBlock (
         vtxI.getIndex (), vtxJ.getIndex ());

      double w_ij = blk.get (0, 0);
      w_ij *= w_ij;

      return w_ij;
   }


   /**
    * 
    * @param e0 first edge
    * @param e1 second edge
    * @return cot (alph), alph is the angle between <tt>e0</tt> and 
    * <tt>e1</tt>
    */
   private static double cotangent (Vector3d e0, Vector3d e1) {
      double dot = e0.dot (e1);
      double el0 = e0.normSquared ();
      double el1 = e1.normSquared ();

      double deno = Math.sqrt (el1 * el0 - dot * dot);

      if (deno < 1E-16) {
         return Double.POSITIVE_INFINITY;
      }

      double cot = dot / deno;

      return cot;
   }


   private static Matrix3x3Block addMapBlock (SparseBlockMatrix Map, 
      double val, int rbi, int cbi) {

      Matrix3x3Block blk = (Matrix3x3Block)Map.getBlock (
         rbi, cbi);
      if (blk == null) {
         blk = new Matrix3x3Block ();
         blk.setDiagonal (val, val, val);
         Map.addBlock (rbi, cbi, blk);
      }
      else {
         blk.m00 += val;
         blk.m11 += val;
         blk.m22 += val;
      }

      return blk;
   }

   /**
    * invert diagonal elements
    * @param Mass lumped mass matrix
    */
   public static void invertMassMatrix (SparseBlockMatrix Mass) {
      for (int i = 0; i < Mass.numBlockRows (); i++) {
         Matrix3x3Block blk = (Matrix3x3Block)Mass.getBlock (i, i);
         blk.m00 = 1.0 / blk.m00;
         blk.m11 = 1.0 / blk.m11;
         blk.m22 = 1.0 / blk.m22;
      }
   }

   /*
    * test Laplacian operator
    */
   public static void main (String [] args) {


      //PolygonalMesh mesh = MeshFactory.createRectangle (1, 1, 5, 5, false);
      //mesh = MeshFactory.createSphere (0.5, 5);
      String path2 = "subjects/swallow_CT04mm/registration/";
      PolygonalMesh mesh = null;
      try {
         mesh = ReadWrite.readMesh (SubjectModel.class, 
            path2 + "cricoid/cricoidSourceMesh.vtk");
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      SparseBlockMatrix Lapl = GeometryOperator.createLaplacianMapMatrix (mesh, true);
      SparseBlockMatrix Mass = GeometryOperator.createVoronoiMassMatrix (mesh);

      VectorNd meanRatio = new VectorNd (mesh.numFaces ());
      int numBad = 0;
      int idx = 0;
      for (Face face : mesh.getFaces ()) {
         Vertex3d [] vts = face.getTriVertices ();
         double val = FEMQualityUtilities.evalTriangleMeanRatio (
            vts[0].getPosition (),
            vts[1].getPosition (), 
            vts[2].getPosition ());
         meanRatio.set (idx, val);
         if (val < 0.1) {
            numBad++;
         }
         idx++;

      }
      System.out.println (meanRatio);
      System.out.println ("min: " + meanRatio.minElement ());
      System.out.println ("number of bad faces: " + numBad);
      System.out.println ("rate of bad faces: " + (double)numBad/(double)mesh.numFaces ());


      VectorNd x = new VectorNd (mesh.numVertices () * 3);
      for (int i = 0; i < mesh.numVertices (); i++) {
         Point3d pnt = new Point3d (mesh.getVertex (i).getWorldPoint ());
         pnt = new Point3d (i, i, i);
         //System.out.print (mesh.getVertex (i).getIndex () + " ");
         x.setSubVector (i*3, pnt);
      }
      //System.out.println ("");

      invertMassMatrix (Mass);
      VectorNd tmp = new VectorNd ();
      Lapl.mul (tmp, x);
      System.out.println ("Laplacian symmetric: " + Lapl.isSymmetric (1E-15));
      VectorNd K_H = new VectorNd ();
      Mass.mul (K_H, tmp);
      //System.out.println (tmp);
      //System.out.println (K_H);

      System.out.println ("degenerate num: " + mesh.numDegenerateFaces ());


      //System.out.println (GeometryOperator.computeMeanCurvatureNormal (mesh, true));

      //SVDecomposition svd = new SVDecomposition (Lapl);
      //System.out.println ("Sigular value: ");
      //System.out.println (svd.getS ());
   }

}
