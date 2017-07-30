package artisynth.models.swallowingRegistrationTool.infoUtilities;

import maspack.matrix.Point3d;

public class EdgeInfo extends ObjInfo{

   Point3d myHeadPoint;
   Point3d myTailPoint;
   
   public EdgeInfo (Point3d head, Point3d tail) {
      myHeadPoint = head;
      myTailPoint = tail;
   }
   
   public EdgeInfo (Point3d head, Point3d tail, String name) {
      myHeadPoint = head;
      myTailPoint = tail;
      setName(name);
   }
   
   public Point3d getHead() {
      return myHeadPoint;
   }
   
   public Point3d getTail() {
      return myTailPoint;
   }
   
   public boolean equal (EdgeInfo info) {
      if (myHeadPoint == info.myHeadPoint &&
      myTailPoint == info.myTailPoint)  {
         return true;
      }
      return false;
   }

   @Override
   public EdgeInfo clone () throws CloneNotSupportedException {
      @SuppressWarnings("unchecked")
      Point3d head = new Point3d (myHeadPoint);
      Point3d tail = new Point3d (myTailPoint);
      EdgeInfo e = new EdgeInfo (head, tail, getName());
      return e;
   }

}
