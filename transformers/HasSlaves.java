package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.ObjInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;

public interface HasSlaves {
   public ArrayList<TransformableGeometry> getSlaves();
   public void addSlave (TransformableGeometry slave);
   public SlaveInfo getSlaveInfo (TransformableGeometry slave);
   public ArrayList<ObjInfo> getSlaveInfos ();
   
   /**
    * set a range for slave point, the unit for <tt>range</tt> is 
    * meter, it's the distance between the furtherest slave point 
    * and the convex hull of the source mesh; 
    *  
    * @param range it's value should depend on registration transformer
    * and must be positive;
    */
   public void setSlaveRange (double range);
   /**
    * @return distance between the furtherest slave point 
    * and the convex hull of the source mesh; 
    */
   public double getSlaveRange ();
}
