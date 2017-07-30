package artisynth.models.swallowingRegistrationTool;

import java.util.Collection;

import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public interface IterativeTask {
   
   /**
    * Parent action will pass iterations to this task.
    * @param actionIteration 
    * @param taskIteration The task iteration in current action iteration;
    */
   public void setIteration (int actionIteration, int taskIteration);
   
   public void setTransformer (RegistrationTransformer tf);

   public void setActiveSlaves (Collection<SlaveInfo> activeSlaves);
   
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
    * @return If this is last task iteration return false otherwise return true;
    */
   public boolean postUpdate (RegistrationManager manager);
}
