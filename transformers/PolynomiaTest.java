package artisynth.models.swallowingRegistrationTool.transformers;

public class PolynomiaTest {

   public static void main (String [] arg) {
      int [] testNList = {3, 5, 7};
      BernsteinPolynomia BPTest = new BernsteinPolynomia(testNList);
      BPTest.codeTest ();
   }

}
