package artisynth.models.swallowingRegistrationTool.utilities;

import maspack.util.Clonable;

public interface DataClonable extends Clonable{
   /**
    * 
    * @return a cloned object which contains data which is needed.
    * @throws CloneNotSupportedException
    */
   public Object CloneForData () throws CloneNotSupportedException;
}
