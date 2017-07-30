package artisynth.models.swallowingRegistrationTool.utilities;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.femmodels.PointSkinAttachment;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.QuadhexElement;
import artisynth.core.femmodels.QuadpyramidElement;
import artisynth.core.femmodels.QuadtetElement;
import artisynth.core.femmodels.QuadwedgeElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank2.GenericModel;
import artisynth.models.subjectFrank.ModelGrowException;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

public class FemModelAgent extends FemMuscleModel{

   protected boolean femPriority = true;

   FemModel3d femClient;

   HashMap <FemNode3d, FemNode3d> NodeMap = 
   new HashMap <FemNode3d, FemNode3d> ();

   // agent muscle fiber  --> client muscle fiber
   protected HashMap <Muscle, Muscle> myBadFibers = new HashMap <Muscle, Muscle> ();
   // agent fem node  --> client fem node
   protected HashMap<FemNode3d, FemNode3d> badNodes = new HashMap<FemNode3d, FemNode3d> ();

   // client master point attachment of node --> node agent
   // PointSkinAttachment not included
   HashMap <PointAttachment, Point> pntNodAtmMap =  new HashMap <PointAttachment, Point> ();
   // client node attachment 
   HashSet <PointAttachment> nodAtmSet = new HashSet <PointAttachment> ();

   protected MechModel clientMech;

   public FemModelAgent (String name) {
      super();
      setName(name);
   }

   public FemModelAgent (String name, FemModel3d fem) {
      this(name);
      femClient = fem;
      represent(femClient);
      resolveHalfExternalMuscles ();
      collectMasterPointNodeAttachment ();
      collectNodeAttachment ();
      
      getSurfaceMesh ();
      resetRestPosition ();
   }

   public FemModelAgent (FemModel3d fem) {
      String name;
      if (fem.getName () != null) {
         name = new String (fem.getName ());
      }
      else {
         name = new String (fem.getClass ().getSimpleName ());
      }
      name = name + "Copy";
      setName(name);
      femClient = fem;
      represent(femClient);
      resolveHalfExternalMuscles ();
      collectMasterPointNodeAttachment ();
      collectNodeAttachment ();
      
      getSurfaceMesh ();
      resetRestPosition ();
   }

   public FemModelAgent (FemMuscleModel fem) {
      String name;
      if (fem.getName () != null) {
         name = new String (fem.getName ());
      }
      else {
         name = new String (fem.getClass ().getSimpleName ());
      }
      name = name + "Copy";
      setName(name);

      femClient = fem;
      represent(fem);
      resolveHalfExternalMuscles ();
      collectMasterPointNodeAttachment ();
      collectNodeAttachment ();
      
      getSurfaceMesh ();
      resetRestPosition ();
   }

   /**
    * Given a FEM model, make a copy for its nodes, elements and 
    * <code>FemMarker</code>s. If it's an instance of 
    * <code>FemMuscleModel</code>, copy its muscle bundle; pass the 
    * copy to this agent; 
    * @param client
    */
   public void represent(FemModel3d client) {
      clear();
      NodeMap.clear ();

      int idx = 0;
      for (FemNode3d node : client.getNodes ()) {
         FemNode3d newNode = new FemNode3d (node.getPosition ());
         newNode.setName ("node" + Integer.toString (idx++));
         addNode (newNode);
         NodeMap.put (node, newNode);
      }

      // element
      for (FemElement3d ele : client.getElements ()) {
         ArrayList<FemNode3d> elementNodeList =  new ArrayList<FemNode3d> ();
         for (FemNode3d node : ele.getNodes ()) {
            elementNodeList.add (NodeMap.get (node));
         }

         FemNode3d [] elementNodes = new FemNode3d[elementNodeList.size ()];
         for (int i = 0; i < elementNodeList.size (); i++) {
            elementNodes[i] = elementNodeList.get (i);
         }

         if (ele instanceof HexElement) {
            HexElement e = new HexElement(elementNodes);
            addElement(e);
         } else if (ele instanceof TetElement) {
            TetElement e = new TetElement(elementNodes);
            addElement (e);
         } else if (ele instanceof PyramidElement) {
            PyramidElement e = new PyramidElement(elementNodes);
            addElement (e);
         } else if (ele instanceof WedgeElement) {
            WedgeElement e = new WedgeElement(elementNodes);
            addElement (e);
         } else if (ele instanceof QuadtetElement) {
            QuadtetElement e = new QuadtetElement(elementNodes);
            addElement (e);
         } else if (ele instanceof QuadpyramidElement) {
            QuadpyramidElement	 e = new QuadpyramidElement(elementNodes);
            addElement (e);
         } else if (ele instanceof QuadwedgeElement) {
            QuadwedgeElement   e = new QuadwedgeElement(elementNodes);
            addElement (e);
         } else if (ele instanceof QuadhexElement) {
            QuadhexElement   e = new QuadhexElement(elementNodes);
            addElement (e);
         }
      }

      // markers 
      for (FemMarker mk : client.markers ()) {
         FemMarker newMk = new FemMarker ();
         newMk.setPosition (mk.getPosition ());

         if (mk.getElement () == null) {
            // not attached to a single element;
            // probably attached to several nodes
            //
            // most possible: a radius based attachment 
            // only handle this situation
            // otherwise throw a new exception
            HashSet <FemNode3d> nodeMasters = new HashSet<FemNode3d>();
            for (DynamicComponent master : mk.getAttachment ().getMasters ()) {
               if (master instanceof FemNode3d) {
                  if (femClient.getNodes ().contains (master)) {
                     nodeMasters.add (NodeMap.get (master));
                  }
                  else {
                     throw new ClassCastException ("Unkown marker master!");
                  }
               }
               else {
                  throw new ClassCastException ("Unkown marker master!");
               }
            }
            addMarker (newMk);
            PointFem3dAttachment att = (PointFem3dAttachment) newMk.getAttachment ();
            att.setFromNodes (newMk.getPosition (), nodeMasters);

         } else {
            // attached to a single elements
            int eIdx = client.getElements ().indexOf (
               mk.getElement ());
            addMarker (newMk, getElement (eIdx));
         }
      }

      // muscle bundle
      if (client instanceof FemMuscleModel) {
         FemMuscleModel femMus = (FemMuscleModel) client;
         RenderableComponentList<MuscleBundle> mbList = femMus.getMuscleBundles ();

         HashMap <Point, Point> mbPntMap = new HashMap <Point, Point>();
         for (MuscleBundle mb : mbList) {
            MuscleBundle newMB = new MuscleBundle ();
            for (Muscle mf : mb.getFibres ()) {
               // old muscle points
               Point pnt1 = mf.getFirstPoint ();
               Point pnt2 = mf.getSecondPoint ();
               Point newPnt1 = null;
               Point newPnt2 = null;
               if (pnt1 instanceof FemMarker) {
                  int m1 = client.markers ().indexOf (pnt1);
                  newPnt1 = markers ().get (m1);
               }
               else {
                  int n1 = client.getNodes ().indexOf (pnt1);
                  newPnt1 = getNode (n1);
               }
               if (pnt2 instanceof FemMarker) {
                  int m2 = client.markers ().indexOf (pnt2);
                  newPnt2 = markers ().get (m2);
               }
               else {
                  int n2 = client.getNodes ().indexOf (pnt2);
                  newPnt2 = getNode (n2);
               }
               // new muscle fiber
               Muscle newMF = new Muscle();
               newMF.setName (mf.getName ());
               newMF.setFirstPoint (newPnt1);
               newMF.setSecondPoint (newPnt2);
               newMB.addFibre (newMF);
            }
            newMB.setName (mb.getName ());
            newMB.setFibresActive (false);
            addMuscleBundle (newMB);
         }

         setDynamicsEnabled (false);
         for (MuscleBundle mb : getMuscleBundles() ) {
            RenderProps.setLineStyle(mb, LineStyle.SPINDLE);
            RenderProps.setLineRadius(mb, 0.0003);
            RenderProps.setLineColor(mb, Color.RED);
            if (mb.getElements().size() > 0) {
               //mb.setDirectionRenderLen(0.7);
               mb.setElementWidgetSize(0.5);
               mb.getRenderProps().setFaceColor( mb.getRenderProps().getLineColor() );
            }
         }
         //RenderProps.setLineColor (this, new Color (0.2f, 0.6f, 1.0f));
      }

   }

   public boolean getFemPriority () {
      return femPriority;
   }

   public void setFemPriority (boolean femPriority) {
      this.femPriority = femPriority;
   }

   public FemModel3d getClient () {
      return femClient;
   }

   public void getNodeMap (HashMap <FemNode3d, FemNode3d> nodeMap) {
      nodeMap.putAll (NodeMap);
   }

   /**
    * If any muscle fiber points are connect with other components, 
    * they are defined as half-external muscle; collect all half
    * exterior muscle fibers and disconnect them to this FEM model 
    * agent; 
    * 
    * <p>
    * Remove and collect isolated nodes(no element dependencies); 
    * defines them as bad nodes; 
    * 
    */
   public void resolveHalfExternalMuscles () {
      myBadFibers.clear ();
      if (! (femClient instanceof FemMuscleModel)) {
         return;
      }
      // TODO: try convex hull method
      FemMuscleModel client  = (FemMuscleModel) femClient;
      // remove bad muscle fiber
      int mbIdx = 0;
      for (MuscleBundle mb : getMuscleBundles ()) {
         for (int i = 0; i < mb.getFibres ().size (); i++) {
            Muscle mf = mb.getFibres ().get (i);
            Point pnt1 = mf.getFirstPoint ();
            Point pnt2 = mf.getSecondPoint ();
            boolean markBad = false;
            if (pnt1 instanceof FemNode3d) {
               FemNode3d node = (FemNode3d) pnt1;
               if (node.numAdjacentElements () == 0) {
                  markBad = true;
               }
            }
            if (pnt2 instanceof FemNode3d) {
               FemNode3d node = (FemNode3d) pnt2;
               if (node.numAdjacentElements () == 0) {
                  markBad = true;
               }
            }
            if (markBad) {
               MuscleBundle clientMb = 
               client.getMuscleBundles ().get (mbIdx);
               Muscle clientMf = clientMb.getFibres ().get (i);
               myBadFibers.put (mf, clientMf);
               mb.removeFibre (mf);
            }
         }
         mbIdx ++;
      }

      badNodes.clear ();
      // remove bad nodes
      int idx = 0;
      for (FemNode3d node : getNodes ()) {
         if (node.numAdjacentElements () == 0) {
            badNodes.put (node, femClient.getNode (idx));
         }
         idx++;
      }
      for (FemNode3d node: badNodes.keySet ()) {
         removeNode (node);
      }

      /*
       * inside mesh method
      for (FemNode3d node : getNodes ()) {
         boolean inside =
         query.isInsideOrientedMesh(mesh, node.getPosition(), 0.0003);
         if (!inside) {
            badNodes.put (node, femClient.getNode (idx++));
         }
      }
      for (FemNode3d node : badNodes.keySet ()) {
         // remove element dependencies
         ArrayList<FemElement3d> elems =
         new ArrayList<>(node.getElementDependencies());
         for (FemElement3d elem : elems) {
            node.removeElementDependency (elem);
         }
         // remove node
         removeNode(node);
      } */
   }

   /**
    * if a fiber has muscle points attached to another components, 
    * defines this fiber as a half-external fiber; all half-external
    * fibers have been disconnected to this FEM agent; 
    * 
    * @return a set of half-external fibers of the client;
    */
   public Set<Muscle> getHalfExternalFibers () {
      HashSet<Muscle> fibers = new HashSet<Muscle> ();
      fibers.addAll (myBadFibers.keySet ());
      return myBadFibers.keySet ();
   }

   /**
    * if a fiber has muscle points attached to another components, 
    * defines this fiber as a half-external fiber; all half-external
    * fibers have been disconnected to this FEM agent; 
    * 
    * @param badFiber a half-external fiber of this FEM agent
    * @return the corresponding half-external muscle fiber of the 
    * client.
    */
   public Muscle getClientHalfExternalFiber (Muscle badFiber) {
      // TODO: check
      return myBadFibers.get (badFiber);
   }

   //TOD: re-write
   /**
    * collect master <code>PointAttachment</code> of the client; 
    * copy point slaves of all <code>PointAttachment</code>s of 
    * client and make new <code>Point</code> agents for them;
    */
   public void collectMasterPointNodeAttachment () {
      LinkedList <DynamicAttachment> atts = new LinkedList<DynamicAttachment> ();
      femClient.getAttachments (atts, 0);

      // get point attachment
      pntNodAtmMap.clear ();

      /*
      for (DynamicAttachment att : atts) {
         if (att instanceof PointAttachment) {
            pntAtmList.add ((PointAttachment)att);
         }
      }*/

      HashMap <Point, Point> clientAgent = new HashMap<Point, Point>();
      for (FemNode3d node : femClient.getNodes ()) {
         if (node.getMasterAttachments () == null) {
            continue;
         }
         for (DynamicAttachment att : node.getMasterAttachments ()) {
            if (att instanceof PointAttachment) {
               if (att instanceof PointSkinAttachment) {
                  continue;
               }
               PointAttachment pntAtm = (PointAttachment)att;
               if ((pntAtm).getPoint () == null) {
                  continue;
               }
               if (! clientAgent.containsKey (pntAtm.getPoint ())) {
                  Point agent = new Point ();
                  Point3d pos = new Point3d (
                     pntAtm.getPoint ().getPosition ());
                  agent.setPosition (pos);
                  clientAgent.put (pntAtm.getPoint (), agent);
               }
               pntNodAtmMap.put (pntAtm, clientAgent.get (pntAtm.getPoint ()));
            }
         }
      }
   }


   /**
    * collect attachments of all element-nodes;
    */
   public void collectNodeAttachment () {
      for (FemNode3d node : femClient.getNodes ()) {
         // ignore nodes outside of surface mesh
         if (node.numAdjacentElements () == 0) {
            continue;
         }
         if (node.getAttachment () == null) {
            continue;
         }
         nodAtmSet.add (
            (PointAttachment)node.getAttachment ());
      }
   }

   /**
    * @return a linked list of master <code>PointAttachment</code>s of 
    * the FEM client;
    */
   public LinkedList <PointAttachment> getPointNodeAttachment () {
      LinkedList <PointAttachment> atts = new LinkedList<PointAttachment> ();
      atts.addAll (pntNodAtmMap.keySet ());
      return atts;
   }

   public PointList <Point> getPointNodeAttachmentSlaves () {
      PointList <Point> pnts = new PointList <Point> (Point.class);
      for (PointAttachment pntAtt : pntNodAtmMap.keySet ()) {
         Point pnt = pntAtt.getPoint ();
         if (! pnts.contains (pnt)) {
            pnts.add (pnt);
         }
      }
      return pnts;
   }

   public PointList <Point> getPointNodeAttachmentExternalIsolatedSlaves () {
      PointList <Point> pnts = getPointNodeAttachmentSlaves ();
      for (Point pnt : pnts) {
         // remove internal marker
         if (pnt instanceof FemMarker) {
            pnts.remove (pnt);
         }
         // remove structured nodes
         if (pnt instanceof FemNode3d) {
            FemNode3d node = (FemNode3d) pnt;
            if (node.numAdjacentElements () != 0) {
               pnts.remove (pnt);
            }
         }
      }
      return pnts;
   }

   public PointList<Point> getPointNodeAttachmentExternalIsolatedSlaveAgents () {
      PointList <Point> pnts = new PointList <Point> (Point.class);
      for (PointAttachment pntAtt : pntNodAtmMap.keySet ()) {
         Point agent = pntNodAtmMap.get (pntAtt);
         if (pnts.contains (agent)) {
            continue;
         }
         Point pnt = pntAtt.getPoint ();
         // remove internal marker
         if (pnt instanceof FemMarker) {
            continue;
         }
         // remove structured nodes
         if (pnt instanceof FemNode3d) {
            FemNode3d node = (FemNode3d) pnt;
            if (node.numAdjacentElements () != 0) {
               continue;
            }
         }
         pnts.add (agent);
      }
      return pnts;
   }

   /**
    * @return a linked list of <code>Point</code> agents of 
    * all all <code>PointAttachment</code> point slaves;
    */
   public PointList <Point> getPointNodeAttachmentSlaveAgents () {
      PointList <Point> pntAtmAgent = new PointList <Point> (Point.class);
      for (Point pnt : pntNodAtmMap.values ()) {
         if (! pntAtmAgent.contains (pnt)) {
            pntAtmAgent.add (pnt);
         }
      }
      return pntAtmAgent;
   }


   /**
    * Some structured nodes of client FE mesh are attached to 
    * other external components, return the agents of these 
    * nodes;
    * @return attached-node agents
    */
   public PointList<Point> getAttachedFemNodeAgents () {
      PointList <Point> pntAtmAgent = new PointList <Point> (Point.class);
      for (PointAttachment att : nodAtmSet ) {
         pntAtmAgent.add (NodeMap.get (att.getPoint ()));
      }
      return pntAtmAgent;
   }

   /**
    * generate a new surface mesh for the FEM agent; And triangulates
    * the new surface mesh; 
    * @param client
    * @return a new triangular surface mesh of the FEM client; 
    */
   public static PolygonalMesh makeSurfaceMeshAgent (FemModel3d client) {
      FemModelAgent agent = new FemModelAgent ("agent");
      agent.represent (client);
      agent.invalidateSurfaceMesh ();
      agent.setAutoGenerateSurface (true);
      PolygonalMesh mesh = agent.getSurfaceMesh ();
      // triagulate 
      return (PolygonalMesh)MeshModelAgent.makeMeshAgent (mesh);
   }

   /**
    * generate a new surface mesh for this agent; 
    * And triangulates the new surface mesh; 
    * @return 
    * a new separate triangular surface mesh of this FEM agent. 
    */
   public PolygonalMesh regenerateSurfaceMeshAgent () {
      invalidateSurfaceMesh ();
      setAutoGenerateSurface (true);
      PolygonalMesh mesh = getSurfaceMesh ();
      return (PolygonalMesh)MeshModelAgent.makeMeshAgent (mesh);
   }

   /**
    * regenerate a new surface mesh for this FEM agent, and 
    * render this surface according to <code>SurfaceRender</code>; 
    * @param sr <code>SurfaceRender</code>;
    * it can be {@link SurfaceRender#Shaded}, {@link SurfaceRender#Stress} 
    * or {@link SurfaceRender#Strain}, otherwise do nothing.
    */
   public void renderAgentSurface (SurfaceRender sr) {
      invalidateSurfaceMesh ();
      setAutoGenerateSurface (true);
      getSurfaceMesh ();
      if (sr == SurfaceRender.Shaded) {
         setSurfaceRendering (SurfaceRender.Shaded);
         RenderProps.setLineColor (this, Color.BLUE);
         RenderProps.setFaceColor (this, new Color (0.5f, 0.5f, 1f));
      }
      else if (sr == SurfaceRender.Strain) {
         setSurfaceRendering (sr);
      }
      else if (sr == SurfaceRender.Stress) {
         setSurfaceRendering (sr);
         setStressPlotRanging(Ranging.Auto);
      }
   }

   /**
    * remove all master <code>PointAttachment<code> for client except  
    * for <code>PointSkinAttachment</code>; Possible slave point types
    * include <code>Marker</code>, <code>FemNode3d</code>;
    * 
    * @param mech parent <code>MechModel<code>
    */
   public void detachPointsToClientMesh (MechModel mech) {
      for (PointAttachment att : pntNodAtmMap.keySet ()) {
         mech.detachPoint (att.getPoint ());
      }
   }

   /**
    * If a node of other FEM volume mesh attached to the client structured
    * nodes, detach it;
    * @param mech parent <code>MechModel<code>
    */
   public void detachExternalFemNodesToClientMesh (MechModel mech) {
      for (PointAttachment att : pntNodAtmMap.keySet ()) {
         if (att.getPoint() instanceof FemNode3d) {
            FemNode3d extNode = (FemNode3d)att.getPoint ();
            if (extNode.numAdjacentElements () != 0) {
               mech.detachPoint (extNode);
            }
         }
      }
   }


   /**
    * detach internal element-nodes for client; 
    * 
    * @param mech parent <code>MechModel<code>
    */
   public void detachClientNodesToExterior (MechModel mech) {
      for (PointAttachment att : nodAtmSet) {
         mech.detachPoint (att.getPoint ());
      }
   }

   public void resetClientNodeFrameAttachment () {
      for (PointAttachment att : nodAtmSet) {
         if (att instanceof PointFrameAttachment) {
            PointFrameAttachment patt = (PointFrameAttachment) att;
            Frame frame = patt.getFrame ();
            Point3d loc = att.getSlave ().getPosition ();
            loc.inverseTransform (frame.getPose ());
            patt.setLocation (loc);
         }
      }
   }

   /**
    * if master fem is close enough reset the attachment 
    * other detach it;
    * @param mech
    */
   public void resetClientNodeFemAttachment (MechModel mech) {
      for (PointAttachment att : nodAtmSet) {
         if (att instanceof PointFem3dAttachment) {
            PointFem3dAttachment patt = (PointFem3dAttachment) att;

            // if master fem is close enough reset attchement
            // otherwise detach it;
            FemNode node = patt.getNodes ()[0];
            if (node == null) {
               throw new ModelGrowException (
               "Unknown node fem attachment!");
            }

            boolean flag = false;
            CompositeComponent parent = node.getParent ();
            do  {
               if (parent instanceof FemModel3d) {
                  FemModel3d fem = (FemModel3d) parent;

                  Point3d loc = new Point3d ();
                  Point3d pnt = new Point3d ();
                  att.getPoint ().getPosition (pnt);
                  FemElement3d ele = fem.findNearestElement (loc, pnt);
                  FemNode3d nearNode = fem.findNearestNode (pnt, 2E-6);
                  
                  if (nearNode != null) {
                     patt.setFromNodes (pnt, new FemNode [] {nearNode});
                  }
                  else if (ele != null) {
                     if (loc.distance (pnt) <= 2E-6) {
                        patt.setFromElement (pnt, ele);
                     }
                     else {
                        mech.detachPoint (patt.getPoint ());
                     }
                  }
                  else {
                     mech.detachPoint (patt.getPoint ());
                  }
                  flag = true;
                  break;
               }
               parent = parent.getParent ();
            }while ( parent != null);

            if (!flag) {
               throw new ModelGrowException (
               "Unknown node fem attachment!");
            }
         }
      }
   }

   //TODO
   public void positionContainedBadNodes () {
      for (FemNode3d agent :  badNodes.keySet()) {
         PointFrameAttachment att = null;
         FemNode3d node = badNodes.get (agent);
         if (node.getAttachment () instanceof PointFrameAttachment) {
            att = (PointFrameAttachment)node.getAttachment ();
         }
         else {
            continue;
         }
         
         FemElement3d ele = femClient.findContainingElement (node.getPosition ());
         if (ele == null) {
            continue;
         }

         Point3d pr = new Point3d ();
         //System.out.println ("node: " + node.getPosition ());
         transformPoint (pr, node.getPosition ());
         Point3d loc = new Point3d (pr);
         
         //System.out.println ("pr: " + pr);

         Frame frame = att.getFrame ();
         loc.inverseTransform (frame.getPose ());
         node.setPosition (pr);
         
         att.setLocation (loc);
         //System.out.println ("node1: " + node.getPosition ());
      }
   }


   /**
    * If a node of other FEM volume mesh attached to the client structured
    * nodes, detach it;
    * <p>
    * detach internal element-nodes for client; 
    * 
    * @param mech parent <code>MechModel<code>
    */
   public void detachClientToExterior (MechModel mech) {
      detachExternalFemNodesToClientMesh(mech);
      detachClientNodesToExterior (mech);
   }


   /**
    * @param mech parent <code>MechModel<code>
    */
   public void setClientMechModel (MechModel mech) {

      if (! mech.models ().contains (femClient)) {
         //throw new NullPointerException ("Client not in this mech model!");
      }
      clientMech = mech;
   }

   /**
    * Detach the FEM client to other components properly, see {@link FemModelAgent
    * #detachClientToExterior};
    * Then pass node positions of this agent back to the client; ignore isolated nodes;
    * No topology change is allowed;
    * 
    */
   public void commit () {
      if (clientMech == null) {
         throw new ImproperStateException ("client mech model not set");
      }

      // remove attachments
      //detachExternalFemNodesToClientMesh (clientMech);
      //detachClientNodesToExterior (clientMech);

      femClient.resetRestPosition ();
      int idx = 0;
      
      if (femPriority) {
         positionContainedBadNodes ();
      }
      // nodes
      for (FemNode3d node : getNodes ()) {
         if (badNodes.containsKey (node)) {
            continue;
         }
         else {
            Point3d pnt3d = new Point3d (node.getPosition ());
            femClient.getNode (idx).setPosition (pnt3d);
         }
         idx++;
      }
      
      resetClientNodeFrameAttachment ();
      
      // reset rest position of FEM nodes
      for (FemNode3d node : femClient.getNodes ()) {
        node.resetRestPosition ();
      }
   }


   /**
    * This method would do {@link FemModelAgent#commit} and change physical 
    * properties of the client.
    * 
    * <p>
    * <code>FemMarker</code>s and other external <code>Point</code> attach to this FEM
    * client will be deformed naturally by this FEM when state are updated.
    * <p>
    * if the parent <code>MechModel</code> not set properly, throw a new exception;
    */
   public void grow () {
   // reset rest position of FEM nodes
      for (FemNode3d node : femClient.getNodes ()) {
         node.resetRestPosition ();
      }
      
      resetClientNodeFemAttachment (clientMech);

      // reset rest position of FEM nodes
      for (FemNode3d node : femClient.getNodes ()) {
         node.resetRestPosition ();
      }
   }


   public void commitAndGrow () {
      commit();
      grow();
   }


   public void writeThisAgent (Class<?> obj, String folder) {
      String femPath = 
      ArtisynthPath.getSrcRelativePath (obj, 
      "/geometry/subjectSpecificModeling/");
      MechModel mech = new MechModel ();
      mech.addModel (this);
      GenericModel.writeAllGeometries (femPath+folder, mech);
      System.out.println ("write done!");
   }

   /**
    * Pass the position information of the <tt>fem</tt> volume mesh 
    * nodes to this agent in order; 
    * If the number of nodes of <tt>fem</tt> not the same as the  
    * agent FEM mesh throw a new exception.
    * @param fem <code>FemModel3d</code> contains the volume mesh
    * data; 
    */
   public void copyVolumeMesh (FemModel3d fem) {
      if (fem.getNodes ().size () != getNodes().size ()) {
         throw new ImproperSizeException ("Incompatible mesh data!");
      }
      int idx = 0;
      for (FemNode3d node : getNodes ()) {
         node.setPosition (fem.getNode (idx++).getPosition ());
      }
   }

   PointList <Point> attAgentRenderHolder = new PointList<Point>(Point.class);
   /**
    * render attachment point except for marks and structured nodes
    */
   public void renderExternalIsolatedSlavePointAgents (RootModel root) {
      if (attAgentRenderHolder != null) {
         if (root.renderables ().contains (attAgentRenderHolder)) {
            return;
         }
      }
      attAgentRenderHolder.clear ();
      //TODO
   }

   public void updateExternalPointNodeAttachmentIsolateSlaveAgents () {
   }

   public void transformPoint (Point3d pr, Point3d pnt) {
      Point3d rpos = new Point3d(pnt);
      FemElement3d restElem = femClient.findContainingElement(rpos);
      if (restElem == null) {
         Point3d newLoc = new Point3d();
         restElem = femClient.findNearestSurfaceElement(newLoc, rpos);
      }
      Vector3d coords = new Vector3d();
      if (!restElem.getNaturalCoordinates (coords, rpos)) {
         //System.out.println ("warning...");
      }
      IntegrationPoint3d ipnt =
         IntegrationPoint3d.create (restElem, coords.x, coords.y, coords.z, 1);
      Matrix3d invJ0 = new Matrix3d();
      double detJ0 =
         IntegrationData3d.computeRestJacobian (
            invJ0, ipnt.getGNs (), restElem.getNodes());
      if (detJ0 <= 0) {
         //System.out.println ("warning...");
      }

      int elemIdx = femClient.getElements().indexOf(restElem);
      FemElement3d elem = getElements().get(elemIdx);

      if (pr != null) {
         Point3d pos = new Point3d();
         ipnt.computePosition (pos, elem.getNodes());
         pr.set (pos);
      }
   }



}
