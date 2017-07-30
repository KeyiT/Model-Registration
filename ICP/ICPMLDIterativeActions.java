package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.Collection;
import java.util.List;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.IterativeActions;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.VectorNd;

public class ICPMLDIterativeActions implements IterativeActions{

   protected MeshBasedLinearDeformer myDeformer;
   protected MLDICPFormulator myForm;
   protected RegistrationManager myManager;
  

   @Override
   public RegistrationTransformer pickAction (
      List<RegistrationTransformer> tfs) {
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof MeshBasedLinearDeformer) {
            myDeformer = (MeshBasedLinearDeformer)tf;
            break;
         }
      }
      return myDeformer;
   }

   @Override
   public boolean addSlave (Collection<TransformableGeometry> slaves) {
      return false;
   }

   @Override
   public void pickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      // do nothing
   }
   
   private double aw = 1.0;
   private double raw = aw;

   @Override
   public boolean preAction (RegistrationManager manager) {
      
      myForm = new MLDICPFormulator ();
      myForm.setDimension (myDeformer.getSize ());
      myForm.setCloudMap (manager.getCloudMap ());
      myForm.setTransformer (myDeformer);
      
      if (raw > 0) {
         SparseBlockMatrix K = myDeformer.getStiffnessMatrix ();
         if (K == null) {
            K = myDeformer.createStiffnessMatrix (true);
         }
         K = new SparseBlockMatrix (K);
         VectorNd F = new VectorNd (myDeformer.creatTargetForce (true));
         
         K.scale (raw);
         F.scale (-raw);
         
         myForm.addQuadraticTerm (K);
         myForm.addPropotionalTerm (F);
      }
     
      return true;
   }

   @Override
   public Formulator getFormulator () {
      return myForm;
   }

   @Override
   public boolean postOptimize (
      RegistrationManager manager, boolean optimized) {
      return true;
   }

   @Override
   public boolean postUpdate (RegistrationManager manager) {
      return true;
   }
   
   
   public void setRigidty (double rigidity) {
      aw = rigidity;
   }
   
   public double getRigidity () {
      return aw;
   }

   public double getActualRigidity () {
      return raw;
   }
   
   protected void setActualRigidity (double rigidity) {
      raw = rigidity;
   }
}
