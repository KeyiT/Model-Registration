package artisynth.models.swallowingRegistrationTool.infoUtilities;

import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public class TransformerSlaveInfo 
<T extends RegistrationTransformer> extends ObjInfo{
   
   protected T myMaster;
   protected SlaveInfo mySlaveInfo;

   public void setSlaveInfo (SlaveInfo parent) {
      mySlaveInfo = parent;
   }
   
   public SlaveInfo getSlaveInfo () {
      return mySlaveInfo;
   }
   
   public T getTransformer () {
      return myMaster;
   }
   
   public void setTransformer (T master) {
      myMaster = master;
   }

}
