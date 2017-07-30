package artisynth.models.swallowingRegistrationTool.utilities;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.modelbase.ControllerBase;

/**
 * 
 * Controller for rendering FEM mesh quality; In each time step, 
 * this class will evaluate the quality(metric: mean ratio) of every
 * elements. 
 * 
 * <p>
 * Green elements have good quality, red have bad, blue means 
 * inverted;
 * 
 * 
 * @author KeyiTang
 *
 */
public class FEMQualityRenderer extends ControllerBase {
   
   FemModel3d myFem;
   double [] myThs;
   FEMQualityUtilities femQU = new FEMQualityUtilities ();
   
   public FEMQualityRenderer (FemModel3d fem) {
      myFem = fem;
      myThs = new double [] {0.1, 0.5};
      fem.setElementWidgetSize (0.75);
   }

   @Override
   public void apply (double t0, double t1) {
      femQU.renderMeanRatioForElements (myFem, myThs);
   }
   
   /**
    * Each elements in map must in the range of [0, 1]; 
    * Map array must be an ascending series.
    */
   public void setColorMap (double [] map) {
      myThs = map.clone ();
   }

}