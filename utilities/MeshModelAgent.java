package artisynth.models.swallowingRegistrationTool.utilities;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.ArrayList;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.PointSkinAttachment;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.femmodels.SkinMeshBody.FemModelInfo;
import artisynth.core.femmodels.SkinMeshBody.FrameInfo;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.workspace.RootModel;
import artisynth.models.subjectFrank.ModelGrowException;
import artisynth.models.swallowingRegistrationTool.transformers.SOFFD3d;
import artisynth.models.swallowingRegistrationTool.transformers.SOFFD3d.BasePositionMethod;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class MeshModelAgent extends FixedMeshBody{

   protected RigidBody rbClient;
   protected MeshComponent mcClient;

   public static enum ClientType {
      RigidBody,
      MeshComponent,
      Unknown
   }

   protected ClientType myClientType = ClientType.Unknown;

   protected MeshBase meshClient;
   private MeshBase oldMeshAgent;

   // attachment --> slave agent
   protected HashMap <PointFrameAttachment, Point> pntFrmMap = 
   new HashMap <PointFrameAttachment, Point>();
   protected MechModel clientMech;
   
   HashMap <Vertex3d, Face []> skinMeshMap = new HashMap <Vertex3d, Face []> ();
   HashMap <Vertex3d, FemModel3d> skinFemMap = new HashMap <Vertex3d, FemModel3d> ();

   private SOFFD3d myFFD;

   public MeshModelAgent () {
      super ();
   }

   public MeshModelAgent (String name) {
      super (name);
   }

   public MeshModelAgent (MeshComponent body) {
      represent (body);
   }

   public MeshModelAgent (RigidBody body) {
      represent (body);
   }
   
   public MeshModelAgent (PolygonalMesh mesh) {
      represent (mesh);
   }

   /**
    * Given a rigid body <tt>client</tt> copy its mesh and 
    * pose in world coordinate, set them to this agent; save
    * its <code>PointFrameAttachment</code>s and make a copy
    * of the slave points associate with these 
    * <code>PointFrameAttachment</code>s.
    * 
    * @param client
    */
   public void represent (RigidBody client) {
      myClientType = ClientType.RigidBody;
      rbClient = client;
      mcClient = null;
      meshClient = client.getMesh ();
      setPose(client.getPose ());
      setMesh(makeMeshAgent (meshClient));
      getMesh().setFixed (false);
      if (getName() == null) {
         String name;
         if (client.getName () != null) {
            name = new String (client.getName ());
         }
         else {
            name = new String (client.getClass ().getSimpleName ());
         }
         name = name + "Agent";
         setName(name);
      }
      collectPointFrameAttachments ();
      generateSOFFD3dForIsolatedAttachment ();
   }

   /**
    * Given a mesh component <tt>client</tt> copy its mesh and 
    * pose in world coordinate, set them to this agent;
    * @param client
    */
   public void represent (MeshComponent client) {
      myClientType = ClientType.MeshComponent;
      mcClient = client;
      rbClient = null;
      meshClient = client.getMesh ();
      if (!meshClient.meshToWorldIsIdentity ()) {
         setPose (meshClient.getMeshToWorld ());
      };
      setMesh(makeMeshAgent (meshClient));
      getMesh().setFixed (false);
      if (getName () == null) {
         String name;
         if (client.getName () != null) {
            name = new String (client.getName ());
         }
         else {
            name = new String (client.getClass ().getSimpleName ());
         }
         name = name + "Agent";
         setName(name);
      }
      
      if (mcClient  instanceof SkinMeshBody) {
         collectSkinMaster () ;
      }

   }

   public void represent (MeshBase mesh) {
      myClientType = ClientType.Unknown;
      mcClient = null;
      mcClient = null;
      meshClient = mesh;
      if (!meshClient.meshToWorldIsIdentity ()) {
         setPose (meshClient.getMeshToWorld ());
      }
      setMesh(makeMeshAgent (meshClient));
      getMesh().setFixed (false);
      
      if (getName() == null) {
         String name;
         if (meshClient.getName () != null) {
            name  = new String (meshClient.getName ());
            setName (name + "Agent");
         }
      }
      setFixed (false);
      
   }
   
  

   public RenderableComponentBase getClient () {
      if (myClientType == ClientType.MeshComponent) {
         return mcClient;
      }
      else if(myClientType == ClientType.RigidBody) {
         return rbClient;
      }
      else {
         return null;
      }
   }

   public ClientType getClientType () {
      return myClientType;
   }

   /**
    * Collect all <code>PointFrameAttachment</code>s of <code>RigidBody
    * </code> client; 
    * 
    * @return if client is <code>RigidBody</code> return true; otherwise 
    * return false;
    */
   protected boolean collectPointFrameAttachments () {
      // client must be rigid body
      if (myClientType != ClientType.RigidBody) return false;

      pntFrmMap.clear ();

      if (rbClient.getMasterAttachments () == null) {
         return true;
      }
      for (DynamicAttachment att : rbClient.getMasterAttachments ()) {
         if (att instanceof PointFrameAttachment) {
            PointFrameAttachment pntAtt = (PointFrameAttachment)att;
            Point pnt = pntAtt.getPoint ();
            if (pnt == null) {
               throw new NullPointerException ("Unkown point attachment slave");
            }
            Point pntCopy = new Point ();
            Point3d pnt3d = new Point3d (
               pnt.getPosition ().x, 
               pnt.getPosition ().y,
               pnt.getPosition ().z);
            pntCopy.setPosition (pnt3d);
            pntFrmMap.put (pntAtt, pntCopy);
         }
      }
      return true;
   }

   /**
    * 
    * @return <code>PointFrameAttachment</code> of client; 
    * if client is not <code>RigidBody</code>, return null
    */
   public ArrayList<PointFrameAttachment> getPointFrameAttachments () {
      ArrayList <PointFrameAttachment> atts = 
      new ArrayList <PointFrameAttachment> ();
      atts.addAll (pntFrmMap.keySet ());
      return atts;
   }

   /**
    * get the slave point of all <code>PointFrameAttachment</code> of 
    * the client; 
    * @return slave point; 
    * if client is not <code>RigidBody</code>, return null
    */
   public ArrayList<Point> getPointFrameAttachmentSlaves () {
      // client must be rigid body
      if (myClientType != ClientType.RigidBody) return null;

      ArrayList<Point> pnts = new ArrayList<Point> ();
      for (PointFrameAttachment att: pntFrmMap.keySet ()) {
         pnts.add (att.getPoint ());
      }
      return pnts;
   }

   /**
    * retrieve <code>PointFrameAttachment</code>s whose slave point 
    * which are not structured nodes;
    * @param femPriority when it's true, if the isolated node is inside of its parent
    * FEM, ignore it. 
    * @return
    */
   public ArrayList<PointFrameAttachment> getIsolatedPointFrameAttachments (boolean femPriority) {
      ArrayList<PointFrameAttachment> list = new ArrayList<PointFrameAttachment> ();
      for (PointFrameAttachment att: pntFrmMap.keySet ()) {
         if (att.getPoint () instanceof FemNode3d) {
            FemNode3d node = (FemNode3d) att.getPoint ();
            // isolated nodes
            if (node.numAdjacentElements () == 0) {
               if (femPriority) {
                  FemModel3d fem = getParentFem (node);
                  FemElement3d ele = fem.findContainingElement (node.getPosition ());
                  if (ele != null) {
                     continue;
                  }
               }
               list.add (att);
            }
         }
         else {
            // other kind of points
            list.add (att);
         }
      }
      return list;
   }
   
   // TODO: present

   
   /**
    * Collect all <code>PointFrameAttachment</code>s of <code>RigidBody
    * </code> client; 
    * 
    * @return if client is <code>RigidBody</code> return true; otherwise 
    * return false;
    */
   protected boolean collectSkinMaster () {
      // client must be skin mesh
      if (myClientType != ClientType.MeshComponent) return false;
      if (! (mcClient instanceof SkinMeshBody)) return false;
      
      SkinMeshBody skin = (SkinMeshBody) mcClient;

      List<FemModelInfo> femInfos = skin.getAllFemModelInfo ();
      List<FrameInfo> frameInfos = skin.getAllFrameInfo ();

      for (Vertex3d vtx : meshClient.getVertices ()) {
         
         
         Point3d pos = new Point3d ();
         vtx.getWorldPoint (pos);
         Point3d near = new Point3d ();
         double dis = Double.MAX_VALUE;
         
         // master
         FemModel3d femM = null;
         Face faceM = null;
         
         for (FemModelInfo info : femInfos) {
            FemModel3d fem = info.getFemModel ();
            Point3d loc = new Point3d ();
            FemElement3d ele = fem.findNearestElement (loc, pos);
            if (ele != null) {
               double d = loc.distance (pos);
               if (d < dis) {
                  near.set (loc);
                  femM = fem;
                  dis = d;
               }
            }
         }
         
         for (FrameInfo info : frameInfos) {
           if (info.getFrame () instanceof RigidBody) {
              RigidBody rb = (RigidBody) info.getFrame ();
              PolygonalMesh mesh = rb.getMesh ();
              BVFeatureQuery bv = new BVFeatureQuery ();
              Point3d loc = new Point3d ();
              Face face = bv.nearestFaceToPoint (loc, null, mesh, pos);
              double d = loc.distance (pos);
              if (d < dis) {
                 near.set (loc);
                 femM = null;
                 faceM = face;
                 dis = d;
              }
           }
           else {
              // don't know what to do yet
           }
         }
         
         if (femM != null) {
            skinFemMap.put (vtx, femM);
         }
         else {
            Face [] faces = new Face [2];
            faces [1] = faceM;
            faces [0] = new Face (0);
            faces [0].set (faceM.getVertices (), faceM.numVertices (), true);
            skinMeshMap.put (vtx, faces);
         }
      }

      return true;
   }
   
   
   private FemModel3d getParentFem (FemNode3d node) {
      
      CompositeComponent parent = node.getParent ();
      FemModel3d fem = null;
      do  {
         if (parent instanceof FemModel3d) {
            fem = (FemModel3d) parent;
         }

         parent = parent.getParent ();
      }while ( parent != null);
     
      if (fem == null) {
         throw new ModelGrowException (
         "Unknown node fem attachment!");
      }
      
      return fem;
   }

   /**
    * retrieve slave points of attached to this rigidbody which are not 
    * structured nodes;
    * @return
    */
   public PointList<Point> getPointFrameAttachmentIsolatedSlaves () {
      PointList<Point> pnts = new PointList<Point> (Point.class);
      for (Point slave : getPointFrameAttachmentSlaves()) {
         if (slave instanceof FemNode3d) {
            FemNode3d node = (FemNode3d) slave;
            // isolated nodes
            if (node.numAdjacentElements () == 0) {
               pnts.add (node);
            }
         }
         else {
            // other kind of points
            pnts.add (slave);
         }
      }
      return pnts;
   }

   /**
    * 
    * @return
    * If client is <code>RigidBody</code> return a copy of slave points
    * of its <code>PointFrameAttachment</code>; otherwise return null;
    */
   public PointList<Point> getPointFrameAttachmentSlaveAgents () {
      // client must be rigid body
      if (myClientType != ClientType.RigidBody) return null;

      PointList <Point> agents = new PointList <Point> (Point.class);
      agents.addAll (pntFrmMap.values ());
      return agents;
   }

   /**
    * retrieve agents of slave points of attached to this rigidbody which 
    * are not structured nodes;
    * @return
    */
   public PointList <Point> getPointFrameAttachmentIsolatedSlaveAgents () {
      // client must be rigid body
      if (myClientType != ClientType.RigidBody) return null;

      PointList <Point> agents = new PointList <Point> (Point.class);
      for (PointFrameAttachment att : pntFrmMap.keySet ()) {
         if (att.getPoint () instanceof FemNode3d) {
            FemNode3d node = (FemNode3d) att.getPoint ();
            // isolated nodes
            if (node.numAdjacentElements () == 0) {
               agents.add (pntFrmMap.get (att));
            }
         }
         else {
            agents.add (pntFrmMap.get (att));
         }
      }
      return agents;
   }


   /**
    * If mesh of client is a <code>PolygonalMesh</code>, make a new 
    * copy and triangulate it;
    * 
    * @return a new mesh
    */
   public static MeshBase makeMeshAgent (MeshBase client) {
      if (client instanceof PolygonalMesh) {
         PolygonalMesh poly = (PolygonalMesh) client.clone ();
         if (! poly.isTriangular ()) {
            //TODO
            // make sure (myTriQuadCountsValid = false)
            Collection<Face> faces = new HashSet<Face> ();
            Vertex3d v0 = poly.addVertex (0, 0, 0);
            Vertex3d v1 =poly.addVertex (1, 1, 1);
            Vertex3d v2 =poly.addVertex (2, 2, 2);
            faces.add (poly.addFace (v0, v1, v2));
            poly.removeFaces (faces);
            poly.removeVertex (v0);
            poly.removeVertex (v1);
            poly.removeVertex (v2);
            poly.triangulate ();
         }
         return poly;
      }
      return client.clone ();
   }

   /**
    * detach external isolated point to the client. Including isolated
    * <code>FemNode3d</code>s, <code>marker</code>s and other 
    * <code>Point<code>s;
    * 
    * @param mech parent <code>MechModel<code>
    */
   public void detachExternalPointsToClient (MechModel mech) {
      ArrayList <DynamicAttachment> mechAtts = new 
      ArrayList <DynamicAttachment> ();
      mech.getAttachments (mechAtts, 0);

      /*
      for (Point pnt : getPointFrameAttachmentSlaves ()) {
         if (mechAtts.contains (pnt.getAttachment ())) {
            mech.detachPoint (pnt);
         }
      }*/
      for (PointFrameAttachment att : pntFrmMap.keySet ()) {
         if (mechAtts.contains (att)) {
            //System.out.println (att.getPoint ().getNumber ());
            //System.out.println (att.getPoint ().getName ());
            //att.printReferences ();

            mech.detachPoint (att.getPoint ());
            //System.out.println (att.getPoint ().getNumber ());
            //System.out.println (att.getPoint ().getName ());

         }
      }
   }

   /**
    * If external points been detached to this <code>RigidBody</code> 
    * client, add these attachments again; and update <code>PointFrameAttachment</code> 
    * map;
    * @param mech parent <code>MechModel<code>
    */
   public void retachExternalPointsToClient (MechModel mech) {
      ArrayList <DynamicAttachment> mechAtts = new 
      ArrayList <DynamicAttachment> ();
      for (PointFrameAttachment att : pntFrmMap.keySet ()) {
         if (!mechAtts.contains (att)) {
            //System.out.println (att.getPoint ().getClass ().getSimpleName ());
            //System.out.println (att.getNumber ());
            //System.out.println (att.getPoint ().getNumber ());
            mech.addAttachment (att);
         }
      }
      collectPointFrameAttachments ();
   }

   // TODO: update map
   /**
    * If isolated points been detached to this <code>RigidBody</code> 
    * client, add this attachment again; and update <code>PointFrameAttachment</code> 
    * map;
    * @param mech parent <code>MechModel<code>
    */
   public void retachExternalIsolatedPointToClient (MechModel mech) {
      ArrayList <DynamicAttachment> mechAtts = new 
      ArrayList <DynamicAttachment> ();
      mech.getAttachments (mechAtts, 0);
      for (PointFrameAttachment att : getIsolatedPointFrameAttachments (true)) {
         if (!mechAtts.contains (att)) {
            mech.attachPoint (att.getPoint (), rbClient);
         }
      }
   }

   /**
    * If the client is a <code>RigidBody</code>, detach it to external 
    * components; 
    * 
    * @param mech parent <code>MechModel<code>
    */
   public void detachClientFrameToExterior (MechModel mech) {
      if (myClientType == ClientType.RigidBody) {
         mech.detachFrame (rbClient);
      }
   }

   /**
    * @param mech parent <code>MechModel<code>
    */
   public void setClientMechModel (MechModel mech) {
      clientMech = mech;
   }

   /**
    * pass all agents state back to the client; If client <code>MechModel</code>
    * not initialized, throw a new exception;
    */
   public void commit () {
      if (clientMech == null) {
         throw new ImproperStateException (
         "Parent mech model of the client not specified");
      }
      if (myClientType == ClientType.RigidBody) {
         int idx = 0;
         RigidTransform3d Xwm = new RigidTransform3d (
            meshClient.getMeshToWorld ());
         Xwm.invert ();
         meshClient.setFixed (false);
         for (Vertex3d vtx : meshClient.getVertices ()) {
            Point3d pnt = new Point3d ();
            pnt.set (getMesh().getVertex (idx++).getWorldPoint ());
            pnt.transform (Xwm);
            vtx.setPosition (pnt);
         }
         ((PolygonalMesh)meshClient).updateFaceNormals ();
      }
      else if (myClientType == ClientType.MeshComponent){
         if (mcClient instanceof SkinMeshBody) {
            SkinMeshBody skin = (SkinMeshBody) mcClient;
            
         }
      }
      
      //rbClient.setInertiaFromDensity (rbClient.getDensity ());
   }
   
   public void grow () {
      if (myClientType == ClientType.RigidBody) {
         updateIsolatedPointFrameAttachmentSlaveAgents ();
         ArrayList <PointFrameAttachment> atts = getIsolatedPointFrameAttachments (true);
         for (PointFrameAttachment att : atts) {
            Point agent = pntFrmMap.get (att);
            Point3d loc = new Point3d (agent.getPosition ());
            loc.inverseTransform (rbClient.getPose ());
            att.setLocation (loc);
         }
         
         rbClient.updateAttachmentPosStates ();
      }
      else if (myClientType == ClientType.MeshComponent){
         // TODO: 
      }
      
   }

   public void commitAndGrow () {
      if (myClientType == ClientType.RigidBody) {
         // mass center
         /*
         AffineTransform3d Xno = new AffineTransform3d ();
         ArrayList <Point3d> vtxList = new ArrayList <Point3d> ();
         ArrayList <Point3d> agentVtxList = new ArrayList <Point3d> ();
         int idx = 0;
         for (Vertex3d vtx : meshClient.getVertices ()) {
            Vertex3d vtxAgent = getMesh ().getVertex (idx);
            Point3d pnt = new Point3d (vtx.getWorldPoint ());
            Point3d pntAgent = new Point3d (vtxAgent.getWorldPoint ());
            vtxList.add (pnt);
            agentVtxList.add (pntAgent);
         }
         Xno.fit (agentVtxList, vtxList);*/
         commit();
         grow ();
         
         
      }
   }

   /**
    * Pass mesh vertex position information to this agent
    * in order. 
    * If the number of vertices of <tt>rb</tt> mesh is not
    * the same as the mesh of this agent, throw a new 
    * exception.
    * 
    * @param rb <code>RigidBody</code> contains the mesh 
    * data;
    */
   public void copyMesh (RigidBody rb) {

      PolygonalMesh meshData = rb.getMesh ();
      PolygonalMesh mesh = (PolygonalMesh) getMesh ();
      if (meshData.getVertices ().size () != mesh.getVertices ().size ()) {
         throw new ImproperSizeException ("Incompatible mesh data");
      }

      RigidTransform3d Xwm = new RigidTransform3d (
         mesh.getMeshToWorld ());
      Xwm.invert ();
      mesh.setFixed (false);

      int idx = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         Point3d pnt = new Point3d ();
         pnt.set (meshData.getVertex (idx).getWorldPoint ());
         pnt.transform (Xwm);
         vtx.setPosition (pnt);
         idx++;
      }
   }

   /**
    * Pass mesh vertex position information to this agent
    * in order. 
    * If the number of vertices of <tt>meshData</tt> is not
    * the same as the mesh of this agent, throw a new 
    * exception.
    * 
    * @param meshData contains the vertex positions to copy;
    */
   public void copyMesh  (MeshBase meshData) {
      PolygonalMesh mesh = (PolygonalMesh) getMesh ();
      if (meshData.getVertices ().size () != mesh.getVertices ().size ()) {
         throw new ImproperSizeException ("Incompatible mesh data");
      }

      RigidTransform3d Xwm = new RigidTransform3d (
         mesh.getMeshToWorld ());
      Xwm.invert ();
      mesh.setFixed (false);

      int idx = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         Point3d pnt = new Point3d ();
         pnt.set (meshData.getVertex (idx).getWorldPoint ());
         pnt.transform (Xwm);
         vtx.setPosition (pnt);
         idx++;
      }
   }

   /**
    * Set positions for isolated slave points(agents) of external 
    * <code>PointFrameAttachment</code>s in order; 
    * 
    * @param positionData position data matrix, row size must be 
    * the number of isolated attached points, column size must
    * be 3; otherwise throw a new exception; 
    */
   public void setIsolatedPointFrameAttachmentSlaveAgents (
      MatrixNd positionData) {

      PointList<Point> agents = 
      getPointFrameAttachmentIsolatedSlaveAgents ();

      if ((positionData.rowSize () != agents.size ()) || 
      (positionData.colSize () != 3)) {
         throw new ImproperSizeException (
         "Incompatible external point frame attachment data!");
      }

      int idx = 0;
      for (Point agent : agents) {
         Point3d pos = new Point3d ();
         positionData.getRow (idx, pos);
         agent.setPosition (pos);
         idx++;
      }
   }

   private PointList<Point> attAgentRenderHolder;

   /**
    * render isolated slave points(agents) of external 
    * <code>PointFrameAttachment</code>s;
    * 
    * @param root parent root model
    */
   public void renderIsolatedPointFrameAttachmentSlaveAgents (
      RootModel root) {
      if (attAgentRenderHolder != null) {
         if (root.renderables ().contains (attAgentRenderHolder)) {
            return;
         }
      }

      attAgentRenderHolder =
      getPointFrameAttachmentIsolatedSlaveAgents();
      RenderProps.setPointStyle (attAgentRenderHolder, PointStyle.SPHERE);
      RenderProps.setPointRadius (attAgentRenderHolder, 0.001);
      RenderProps.setPointColor (attAgentRenderHolder, Color.GREEN);
      root.addRenderable (attAgentRenderHolder);
   }

   public void disableIsolatedPointFrameAttachmentSlaveAgents (RootModel root) {
      if (attAgentRenderHolder == null) return;
      if (root.renderables ().contains (attAgentRenderHolder)) {
         root.removeRenderable (attAgentRenderHolder);
      }
   }

   /**
    * update isolated slave points(agents) of external 
    * <code>PointFrameAttachment</code>s
    */
   public void updateIsolatedPointFrameAttachmentSlaveAgents () {
      myFFD.updateSlavePoints ();
   }

   private void generateSOFFD3dForIsolatedAttachment () {
      if (!(getMesh() instanceof PolygonalMesh)) {
         throw new ImproperStateException (
         "unable to generate skin behaviour for agent mesh");
      }
      myFFD = new SOFFD3d ((PolygonalMesh)getMesh ());
      myFFD.setBaseMethod (BasePositionMethod.Fixed);
      myFFD.setBaseWeight (0.25);
      myFFD.addSlavePoints (getPointFrameAttachmentIsolatedSlaveAgents());
      myFFD.updateMaps ();
   }
   
   // TODO
   public void updateSkinMesh (Collection<FemModelAgent> agents) {
      if (myClientType != ClientType.MeshComponent) {
         throw new ImproperStateException (
         "Client Not Skin Mesh");
      }
      if (!(mcClient instanceof SkinMeshBody)) {
         throw new ImproperStateException (
         "Client Not Skin Mesh");
      }
      
      SkinMeshBody skin = (SkinMeshBody) mcClient;
      List<FemModelInfo> femInfos = skin.getAllFemModelInfo ();
      List<FrameInfo> frameInfos = skin.getAllFrameInfo ();
      
      Set ens = skinMeshMap.entrySet ();
      Iterator it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<Vertex3d, Face[]> me = (Entry<Vertex3d, Face[]>)it.next ();
         Face face0 = me.getValue () [0];
         Face face1 = me.getValue () [1];
         Vector2d uv = new Vector2d ();
         Point3d pr = new Point3d ();
         face0.computeCoords (me.getKey ().getPosition (), uv);
         face1.computePoint (pr, uv);
         
         me.getKey ().setPosition (pr);
         if (!meshClient.meshToWorldIsIdentity ()) {
            pr.transform (meshClient.XMeshToWorld);
         }
         skin.setBasePosition (me.getKey ().getIndex (), pr);
      }
      
      ens = skinFemMap.entrySet ();
      it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<Vertex3d, FemModel3d> me = (Entry<Vertex3d, FemModel3d>)it.next ();
         for (FemModelAgent agent : agents) {
            if (agent.getClient () == me.getValue ()) {
               Point3d pr = new Point3d ();
               agent.transformPoint (pr, me.getKey ().getWorldPoint ());
               skin.setBasePosition (me.getKey ().getIndex (), pr);
               if (!meshClient.meshToWorldIsIdentity ()) {
                  pr.inverseTransform (meshClient.XMeshToWorld);
               }
               me.getKey ().setPosition (pr);
               break;
            }
         }
      }
      
      meshClient.notifyVertexPositionsModified ();
    
   }

   public SOFFD3d getSOFFD () {
      return myFFD;
   }
}
