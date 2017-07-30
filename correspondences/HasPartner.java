package artisynth.models.swallowingRegistrationTool.correspondences;

/**
 * correspondence-feature for a single object, which has 
 * dynamic partner.
 * 
 * @author KeyiTang
 */
public interface HasPartner <S, P> {
   
   public P getPartner();
   
   public void setPartner (P partner);
   
   public S getSelf ();
   
   public void setSelf (S self);
   
   public void lockRelationship (boolean lock);
   
   public boolean isLocked ();
}
