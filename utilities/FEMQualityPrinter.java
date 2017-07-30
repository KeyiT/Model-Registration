package artisynth.models.swallowingRegistrationTool.utilities;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import artisynth.core.femmodels.FemModel3d;
import artisynth.models.modelOrderReduction.ReadWrite;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;
import maspack.util.NumberFormat;

public class FEMQualityPrinter {

   public static void main (String [] args) {
      
   }
   
   public static void printMeanRatios (FemModel3d fem) {
      FEMQualityUtilities tool = new FEMQualityUtilities ();
      double [] eta = tool.evalMeanRatios (fem);
      System.out.println (new VectorNd (eta));
   }
   
   public static void writeMeanRatios (FemModel3d fem, String fileName) 
   throws IOException {
      //VectorNd etas = new VectorNd (fem.numElements ());
      FEMQualityUtilities tool = new FEMQualityUtilities ();
      //etas.set (tool.evalMeanRatios (fem));
      MatrixNd data = new MatrixNd (fem.numElements (), 1);
      data.set (tool.evalMeanRatios (fem));
      //System.out.println (data);
      ReadWrite.writeMatrixToFile (data, fileName);
      /*
      File fp =  new File (fileName);
      PrintWriter pw = new PrintWriter (fp);
      NumberFormat fm = new NumberFormat ();
      fm.set ("%f");
      etas.write (pw, fm);*/
      
      
      
   }

}
