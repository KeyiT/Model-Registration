package artisynth.models.swallowingRegistrationTool.infoUtilities;

import artisynth.models.swallowingRegistrationTool.utilities.Updatable;

public abstract class ObjInfo implements Updatable{

   protected String myName;

   public void setName(String name) {
      myName = new String (name);
   }

   public String getName() {
      return myName;
   }

   @Override
   public boolean update() {
      return true;
   }

}
