package artisynth.models.swallowingRegistrationTool.utilities;

import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;

public class RegistrationReport {
   public double ErrSrc2Tgt = Double.NaN;
   public double ErrTgt2Src = Double.NaN;
   public double ErrDual = Double.NaN;
   public double [] src2tgtErrs;
   public double [] tgt2srcErrs;
   public double DevSrc2Tgt = Double.NaN;
   public double DevTgt2Src = Double.NaN;
   public double DevDual = Double.NaN;
   public int myIndex = 0;
   
   public RegistrationReport (int index) {
      myIndex = index;
   }
   public RegistrationReport () {
      myIndex = 0;
   }
   
   public double computeErrSrc2Tgt () {
      if (src2tgtErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      
      int numSrc = src2tgtErrs.length;
      double sum = 0;
      for (int i = 0; i < numSrc; i++) {
         sum += src2tgtErrs[i];
      }
      ErrSrc2Tgt = sum / numSrc;
      
      return ErrSrc2Tgt;
   }
   
   public double computeErrTgt2Src () {
      if (tgt2srcErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      
      int numTgt = tgt2srcErrs.length;
      double sum = 0;
      for (int i = 0; i < numTgt; i++) {
         sum += tgt2srcErrs[i];
      }
      ErrTgt2Src = sum / numTgt;
      
      return ErrTgt2Src;
   }
   
   public double computeErrDual () {
      if (tgt2srcErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      if (src2tgtErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      
      int num = tgt2srcErrs.length;
      double sum = 0;
      for (int i = 0; i < num; i++) {
         sum += tgt2srcErrs[i];
      }
      num = src2tgtErrs.length;
      for (int i = 0; i < num; i++) {
         sum += src2tgtErrs[i];
      }
      ErrDual = sum / (tgt2srcErrs.length +  num);
      
      return ErrDual;
   }
   
   public double computeDevSrc2Tgt () {
      if (src2tgtErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      
      int numSrc = src2tgtErrs.length;
      double sum = 0;
      for (int i = 0; i < numSrc; i++) {
         sum += src2tgtErrs[i];
      }
      ErrSrc2Tgt = sum / numSrc;
      
      double DevSum = 0;
      for (int i = 0; i < numSrc; i++) {
         DevSum += Math.pow ((src2tgtErrs [i] - ErrSrc2Tgt), 2);
      }
      double dev = Math.sqrt (DevSum);
      dev = dev / Math.sqrt (numSrc);
      DevSrc2Tgt = dev;
      
      return DevSrc2Tgt;
   }
   
   public double computeDevTgt2Src () {
      if (tgt2srcErrs == null) {
         throw new ImproperStateException ("Null Pointer");
      }
      
      int numTgt = tgt2srcErrs.length;
      double sum = 0;
      for (int i = 0; i < numTgt; i++) {
         sum += tgt2srcErrs[i];
      }
      ErrTgt2Src = sum / numTgt;
      
      double DevSum = 0;
      for (int i = 0; i < numTgt; i++) {
         DevSum += Math.pow ((tgt2srcErrs [i] - ErrTgt2Src), 2);
      }
      double dev = Math.sqrt (DevSum);
      dev = dev / Math.sqrt (numTgt);
      DevTgt2Src = dev;
      
      return DevTgt2Src;
   }
  
}
