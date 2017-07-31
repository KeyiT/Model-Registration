package artisynth.models.swallowingRegistrationTool.main;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

/**
 * 
 * @author KeyiTang
 *
 */
public abstract class CompositeIterativeActionsBase implements IterativeActions{
   
   RegistrationTransformer myTf;
   Set<SlaveInfo> myActiveSlaves = new HashSet <SlaveInfo> ();
   
   int myIteration = 0;
   int myTaskIteration = 0;
   boolean doTask = true;
   
   protected IterativeTask myTask;
   

   @Override
   final public RegistrationTransformer pickAction (
      List<RegistrationTransformer> tfs) {
      
      if (myTaskIteration == 0) {
        myTf = globalPickAction (tfs);
      }
      
      return myTf;
   }

   @Override
   final public boolean addSlave (Collection<TransformableGeometry> slaves) {
      if (myTaskIteration == 0) {
         return globalAddSlave (slaves);
      }
      return false;
   }

   @Override
   final public void pickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      
      if (myTaskIteration == 0) {
         myActiveSlaves.clear ();
         globalPickSlaves (myActiveSlaves, slaves);
      }
      
      activeSlaves.addAll (myActiveSlaves);
   }
   
   
   @Override
   final public boolean preAction (RegistrationManager manager) {
      
      if (myTaskIteration == 0) {
         
         doTask = globalPreaction (manager);
         if (! doTask) {
            return false;
         }
         else {
            myTask = createIterativeTask ();
            if (myTask == null) {
               doTask = false;
               return false;
            }
            myTask.setTransformer (myTf);
            myTask.setActiveSlaves (myActiveSlaves);
         }
      }
      
      myTask.setIteration (myIteration, myTaskIteration);
      return myTask.preAction (manager);
   }

   @Override
   final public Formulator getFormulator () {
      return myTask.getFormulator ();
   }

   @Override
   final public boolean postOptimize (
      RegistrationManager manager, boolean optimized) {
      
      return myTask.postOptimize (manager, optimized);
   }

   @Override
   final public boolean postUpdate (RegistrationManager manager) {
      
      boolean flag = true;
      
      if (!doTask) {
         flag = globalPostUpdate (manager);
         myIteration++;
         myTaskIteration = 0;
         return flag;
      }
      else {
         if (! myTask.postUpdate (manager)) {
            flag = globalPostUpdate (manager);
            myIteration++;
            myTaskIteration = 0;
            return flag;
         }
         else {
            myTaskIteration ++;
         }
      }
      
      return true;
   }
   
   /**
    * specify <code>IterativeTask</code>; This method will be called in {@link #preAction}
    * at beginning of each action iteration; If return null, no optimization would be done,
    * and manager will skip to {@link #postUpdate} and start a new action iteration;
    * @return
    */
   abstract protected IterativeTask createIterativeTask ();
   
   /**
    * pick a transformer in each iteration; Corresponding optimizer will be 
    * used to optimize this transformer in this iteration. 
    * @param tfs A list of transformers stored in parent registration manager 
    * @return one of transformers in <tt>tfs</tt> list, otherwise parent 
    * registration manager would throw a new exception. If return null parent 
    * manager will skip this iteration and do update only. 
    */
   abstract protected RegistrationTransformer globalPickAction (
      List<RegistrationTransformer> tfs);
   
   
   /**
    * add new slaves into the transformer picked in the corresponding iteration;
    * @param slaves slaves returned in this collection will be added to the 
    * transformer if return value is true; 
    * @return if there are new slaves return true otherwise return false; 
    * If return false, parent manager will do nothing even there are slaves in 
    * <tt>slaves<tt>; 
    */
   abstract protected boolean globalAddSlave (
      Collection<TransformableGeometry> slaves);
   
   
   /**
    * 
    * @param activeSlave slaves returned in this collection will be transformed in 
    * this iteration;
    * @param slaves slaves of the picked transformer
    */
   abstract protected void globalPickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves);
   
   
   /**
    * update registration manger before taking registration action
    * @param manager parent registration manager
    * @return If return true, take registration action, otherwise skip to
    * {@link #postUpdate} directly and the subtask of this action iteration will
    * be skipped;
    */
   abstract protected boolean globalPreaction (
      RegistrationManager manager);
   
   
   /**
    * update registration manger after taking registration action
    * @param manager parent registration manager
    * @return If this is last iteration return false otherwise return true;
    */
   abstract protected boolean globalPostUpdate (
      RegistrationManager manager);

   

}
