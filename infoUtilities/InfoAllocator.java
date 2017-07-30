package artisynth.models.swallowingRegistrationTool.infoUtilities;


public interface InfoAllocator <T extends ObjInfo> {
   /**
    * allocate CloudInfo
    * @param resultInfo if not null, return ObjInfo which has been made
    */
   public void allocateInfo(CloudInfo<T> resultInfo) throws ReflectiveOperationException;
   
   /**
    * add infos to cloud info
    * @param resultInfo 
    */
   public void addInfo (CloudInfo<T> resultInfo) throws ReflectiveOperationException;

}
