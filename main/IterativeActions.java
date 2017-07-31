package artisynth.models.swallowingRegistrationTool.main;

import java.util.Collection;
import java.util.List;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.ObjInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public interface IterativeActions {
   /**
    * pick a transformer in each iteration; Corresponding optimizer will be 
    * used to optimize this transformer in this iteration. 
    * @param tfs A list of transformers stored in parent registration manager 
    * @return one of transformers in <tt>tfs</tt> list, otherwise parent 
    * registration manager would throw a new exception. If return null parent 
    * manager will skip this iteration and do update only. 
    */
   public RegistrationTransformer pickAction (List<RegistrationTransformer> tfs);

   /**
    * add new slaves into the transformer picked in the corresponding iteration;
    * @param slaves slaves returned in this collection will be added to the 
    * transformer if return value is true; 
    * @return if there are new slaves return true otherwise return false; 
    * If return false, parent manager will do nothing even there are slaves in 
    * <tt>slaves<tt>; 
    */
   public boolean addSlave (Collection <TransformableGeometry> slaves);

   /**
    * 
    * @param activeSlave slaves returned in this collection will be transformed in 
    * this iteration;
    * @param slaves slaves of the picked transformer
    */
   public void pickSlaves (Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves);
   
   /**
    * update registration manger before taking registration action
    * @param manager parent registration manager
    * @return If return true, take registration action, otherwise skip to
    * {@link #postUpdate} directly;
    */
   public boolean preAction (RegistrationManager manager);
   
   
   /**
    * 
    * @return formulator needed for making cost function
    */
   public Formulator getFormulator ();
   
   
   /**
    * update registration manger before after optimization
    * @param manager parent registration manager
    * @param optimized a true value means transformer has been optimized 
    * otherwise return false;
    * @return If return true, take transformer action, otherwise skip to
    * {@link #postUpdate} directly; if a exception happens whether taking 
    * transformer action will be determined by <tt>optimized</tt>. In later
    * case if <tt>optimized</tt> value is false manager will skip to
    * {@link #postUpdate} directly;
    */
   public boolean postOptimize (RegistrationManager manager, boolean optimized);
   
   
   /**
    * update registration manger after taking registration action
    * @param manager parent registration manager
    * @return If this is last iteration return false otherwise return true;
    */
   public boolean postUpdate (RegistrationManager manager);
}
