package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.LinkedHashMap;
import java.util.List;

import artisynth.core.mechmodels.Point;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;

public abstract class SOFFDWeightsMaker {
   /**
    * 
    * @param points slave points
    * @param mesh undeformed mesh
    * @return map from the list of picked faces to their
    * corresponding weights; if the map size not equal to 
    * the number of slave points then throw an exception.
    */
   public abstract LinkedHashMap <List<Face>, double []> 
   makeReferenceAndWeights (List<Point> points, PolygonalMesh mesh);
}
