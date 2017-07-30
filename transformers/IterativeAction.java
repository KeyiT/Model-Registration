package artisynth.models.swallowingRegistrationTool.transformers;

public interface IterativeAction {
   /**
    * Take iterative action. This method 
    * would be called in advance. 
    * 
    * Every registration manager can have only
    * one iterative action. This action would be 
    * advanced by root model
    * 
    * @return mapping error; this error would be recorded
    * in output probe.
    */
   public double takeIterativeAction();
}
