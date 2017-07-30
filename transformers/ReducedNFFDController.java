package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Collection;
import java.util.List;
import java.util.Set;

import javax.swing.JMenuItem;

import artisynth.core.driver.Main;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class ReducedNFFDController extends ControllerBase implements ActionListener{
   
   ReducedNFFD3d myFFD;
   public final static RenderProps DEFAULT_RENDER_PROPS = new RenderProps ();
   
   public ReducedNFFDController () {
      super ();
      myFFD = new ReducedNFFD3d ();
      createDefaultRenderProps ();
   }
   
   public void initializeNFFD (int [] numCtrlPnts, int [] degrees, 
      MeshBase mesh, Set<Point3d> slaves) {
      myFFD.build (numCtrlPnts, degrees, mesh, slaves);
   }
   
   public void initializeNFFD (int [] numCtrlPnts, int [] degrees, 
      Set<MeshBase> meshes, Set<Point3d> slaves) {
      myFFD.build (numCtrlPnts, degrees, meshes, slaves);
   }
   
   
   //TEST
   public void renderCH (RootModel root) {
      FixedMeshBody body = new FixedMeshBody ();
      PolygonalMesh mesh = new PolygonalMesh ();
      for (NFFD3dControlUnit unit : myFFD.myUnits) {
         unit.generateConvexHull ();
         mesh.addMesh (unit.myCH.getConvexHullMesh ());
         body.setMesh (mesh);
      }
      root.addRenderable (body);
   }
   

   
   public NFFD3d getNFFD () {
      return myFFD;
   }

   @Override
   public void apply (double t0, double t1) {
      myFFD.updateMeshes ();
   }
   
   public void enableGridControl () {
      RootModel root = RootModel.getRoot (this);
      myFFD.enableGridControl (root);
   }
   
   public void enableGridControl (RootModel root) {
      myFFD.enableGridControl (root);
   }
   
// -------------------------------------
   // implement action listener
   // -------------------------------------


   @Override
   public void actionPerformed(ActionEvent event) {
      if (event.getActionCommand().equals ("apply FFD")) {
         applyFFD();
      }
      else if (event.getActionCommand().equals ("undo FFD")) {
         undoFFD();
      }
      else if (event.getActionCommand().equals ("reset FFD grid")) {
         resetFFDGrid();
      }
      else if (event.getActionCommand ().equals ("advance meshes")) {
         advanceMeshes();
      }
   }

   private void applyFFD() {
      resetTime();
      myFFD.updateMeshes();
   }

   private void undoFFD() {
      resetTime();
      myFFD.resetControlGrid ();
      myFFD.updateMeshes();
   }

   private void resetFFDGrid() {
      resetTime();
      myFFD.resetControlGrid ();
      updateRender();
   }

   private void advanceMeshes() {
      resetTime();
      myFFD.advanceMeshInitialStates();
   }

   private void resetTime() {
      RootModel root = RootModel.getRoot (this);
      if (root != null) {
         root.getWayPoint(0).setValid (false);
         Main main = Main.getMain();
         if (main != null) {
            main.reset();
         }
      }
   }

   public boolean getMenuItems(List<Object> items) {
      JMenuItem menuItem;
      menuItem = makeMenuItem ("apply FFD", "deform the model");
      items.add (menuItem);
      items.add (makeMenuItem ("reset FFD grid", "reset the deformation grid"));
      menuItem = makeMenuItem ("undo FFD", "unfo the deformation");
      items.add (menuItem);
      menuItem = makeMenuItem ("advance mesh", "advance mesh material space to current natural state");
      items.add (menuItem);
      return true;
   }   

   public JMenuItem makeMenuItem (String cmd, String toolTip) {
      JMenuItem item = new JMenuItem(cmd);
      item.addActionListener(this);
      item.setActionCommand(cmd);
      if (toolTip != null && !toolTip.equals ("")) {
         item.setToolTipText (toolTip);
      }
      return item;
   }
   
   public void updateRender() {
      updateRender(this);
   }

   public static void updateRender(ModelComponent mc) {
      RootModel root = RootModel.getRoot (mc);
      if (root != null) {
         root.rerender();
      }
   }

   public void prerender (RenderList list) {
      super.prerender (list);
      if (myRenderProps != null) {
         myFFD.getRenderProps ().set (myRenderProps); 
         myFFD.getCtrlPntAgents ().getRenderProps ().set (myRenderProps);
      }
      list.addIfVisible (myFFD);
   }
   
   public RenderProps createRenderProps() {
      RenderProps rp = RenderProps.createPointLineProps (this);
      rp.set (DEFAULT_RENDER_PROPS);
      return rp;
   }
   
   protected final void createDefaultRenderProps () {
      DEFAULT_RENDER_PROPS.setPointColor (new Color (0.2f, 0.6f, 1.0f));
      DEFAULT_RENDER_PROPS.setPointStyle (PointStyle.SPHERE);
      DEFAULT_RENDER_PROPS.setPointRadius (0.001);
      DEFAULT_RENDER_PROPS.setLineColor (new Color (0.2f, 0.6f, 1.0f));
   }
}