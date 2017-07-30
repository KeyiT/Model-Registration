package artisynth.models.swallowingRegistrationTool.transformers;

import artisynth.models.modelOrderReduction.PrintData;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.MatrixBase;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorBase;
import maspack.matrix.VectorNd;

public class BernsteinPolynomia {

   int myDimension = 1; // dimension, set as column size of P matrix [setP]
   int myN = 1; // term number, set based on NList [setNList]
   int [] myNList = {1}; // required input

   protected MatrixNd myP = new MatrixNd (); // set according to input P matrix [setP]
   protected VectorNd myB = new VectorNd ();  // set according to NList, [setB]
   protected VectorNd myXSTU = new VectorNd(); // input


   public BernsteinPolynomia (int [] argNList) {
      // TODO Auto-generated constructor stub
      setNList(argNList);
   }

   /**
    * coefficients of polynomial expansion
    * @param argN gives the total degrees of freedom;
    * @param argV denotes the index of binomial term;  0 <= argV <= argN
    * argN and argV must be nature numbers
    * @return C(n, v)
    */
   public static double getBinomial(int argN, int argV) {
      double temC = 1.0;
      // C(n, v) = Mul (n+1-i)/i
      for (int i = 1; i <= argV; i++) {
         temC = temC * (argN+1-i) / i;
      }
      return temC;
   }

   /**
    * 
    * @param argN gives the total degrees of freedom;
    * @param argV denotes the index of binomial term;  0 <= argV <= argN
    * @param argX independent variable; [0, 1]
    * @return C(n v) * x^v * (1-x)^(n-v)
    */
   public static double getBerBasis(int argN, int argV, double argX) {
      if (argV < 0 || argV > argN) {
         return 0;
      }
      double temB;
      temB = getBinomial(argN, argV);
      temB = temB * Math.pow(argX, argV); 
      temB = temB * Math.pow(1-argX, argN - argV);
      return temB;
   }
   
   /**
    * 
    * @param argN gives the total degrees of freedom;
    * @param argV denotes the index of binomial term;  0 <= argV <= argN
    * @param argX independent variable; [0, 1]
    * @return derivative of b(n, v)
    */
   public static double getBerBasisDerivative(int argN, int argV, double argX) {
      double temB;
      temB = argN * ( getBerBasis(argN-1, argV-1, argX) - getBerBasis(argN-1, argV, argX) );
      return temB;
   }

   /*
    * --------------------------------------------
    * set and get P
    * --------------------------------------------
    */
   /**
    * set Bézier coefficients.
    * P is N x D matrix
    * @param argP N x D matrix
    */
   public void setP(MatrixBase argP) {
      myP.set (argP);
      setDimesion(argP);
   }

   /**
    * set Bézier coefficients.
    * P is N x D matrix
    * @param argP N x D double array
    */
   public void setP(double [][] argP) {
      myP.set (argP);
      setDimesion(argP[0].length);
   }

   /**
    * 
    * @return Bézier coefficients
    */
   public MatrixNd getP() {
      return myP;
   }

   /**
    * Bézier coefficients catenated to form a vector
    * length = numbers of coefficients * dimension
    * @return Bézier coefficients
    */
   public VectorNd getPVec() {
      VectorNd PVec  = new VectorNd ();
      PVec.setSize (myP.colSize ()*myP.rowSize ());
      int k = 0;
      for (int i = 0; i < myP.rowSize (); i++) {
         for (int j = 0; j < myP.colSize (); j++) {
            PVec.set (k++, myP.get (i, j));
         }
      }
      return PVec;
   }

   /*
    * --------------------------------------------
    * set and get B
    * --------------------------------------------
    */

   /**
    * make blending functions
    * return as blending vector
    * 
    * need myNList
    * myNList[i] is a higher order position with respect to myNList[i+1]
    * @param argX
    * @return
    */
   public VectorNd makeBlendingFunctions(VectorBase argX) {
      if (argX.size () != myDimension) {
         throw new ImproperSizeException ("Incompatible dimensions");
      }
      VectorNd lastB = new VectorNd();
      VectorNd temB = new VectorNd();

      lastB.setSize (1);
      lastB.set (0, 1.0);

      for (int i = 0; i < myDimension; i++) {
         temB.setSize (myNList[i]+1); 
         for (int j = 0; j <= myNList[i]; j++) {
            temB.set (j, getBerBasis(myNList[i], j, argX.get (i)) );
         }
         lastB.set (outerProductVector(lastB, temB)); 
      }

      return lastB;
   }

   /**
    * make blending functions
    * return as blending vector
    * 
    * b_v(x) = b_kji...(x)
    * @param argX
    * @return
    */
   public VectorNd makeBlendingFunctions(double [] argX) {
      VectorNd temXV = new VectorNd(argX);
      return makeBlendingFunctions(temXV);
   }

   /**
    * make blending functions
    * return as blending matrix
    * @param argX
    * @return
    */
   public MatrixNd makeBlendingMatrix(VectorBase argX) {
      VectorNd temBV = makeBlendingFunctions(argX);

      return BVec2Mat(temBV);
   }

   /**
    * make blending functions
    * return as blending matrix
    * @param argX
    * @return
    */
   public MatrixNd makeBlendingMatrix(double [] argX) {
      VectorNd temXV = new VectorNd(argX);
      return makeBlendingMatrix(temXV);
   }
   
   /**
    * 
    * @param argX
    * @param dimensionIndex
    * @return
    */
   public VectorNd makeBlendingFunctionsForDerivative(VectorBase argX, int dimensionIndex) {
      if (argX.size () != myDimension) {
         throw new ImproperSizeException ("Incompatible dimensions");
      }
      VectorNd lastB = new VectorNd();
      VectorNd temB = new VectorNd();

      lastB.setSize (1);
      lastB.set (0, 1.0);

      for (int i = 0; i < myDimension; i++) {
         int tmpN = 0;
         
         if (i != dimensionIndex) {
            tmpN = myNList[i] + 1;
         } else {
            tmpN = myNList[i];
         }
         
         temB.setSize (tmpN); 
         for (int j = 0; j < tmpN; j++) {
            temB.set (j, getBerBasis(tmpN-1, j, argX.get (i)) );
         }
         lastB.set (outerProductVector(lastB, temB)); 
      }

      return lastB;
   }


   /**
    * b_v = b_kji
    * set myB
    * need myNList
    * myNList[i] is a higher order position with respect to myNList[i+1]
    * @param argX input location
    */
   public void setB(VectorBase argX) {
      myB.set (makeBlendingFunctions(argX));
   }

   /**
    * b_v = b_kji
    * set myB
    * need myNList
    * myNList[i] is a higher order position with respect to myNList[i+1]
    * @param argX input location
    */
   public void setB(double [] argX) {
      myB.set (makeBlendingFunctions(argX));
   }

   /**
    * 
    * @return Berstein basis in vector form
    */
   public VectorNd getB() {
      return myB;
   }

   /**
    * set Bernstein Basis myB in Matrix form 
    * need set myB first
    * @return Berstein basises in matrix form
    */
   public MatrixNd getBMat() {
      return BVec2Mat(myB);
   }

   /**
    * convert the blending Vector B 
    * to a blending matrix
    * @param B blending vector
    * @return blending matrix
    */
   public MatrixNd BVec2Mat(VectorNd B) {
      if (B.size () != myN) {
         throw new ImproperSizeException ("Incompatible dimensions");
      }

      MatrixNd temBba = new MatrixNd(myDimension, myN*myDimension);
      for (int i = 0; i < B.size (); i++) {
         for (int j = 0; j < myDimension; j++) {
            temBba.set (j, i*myDimension+j, B.get (i));
         }
      }
      return temBba;
   }

   /**
    * M = v1 * v2^T (outer product)
    * @param argV1
    * @param argV2
    * @return packing M into a vector 
    */
   public static VectorNd outerProductVector (VectorNd argV1, VectorNd argV2) {
      int temR = argV1.size ();
      int temC =argV2.size ();
      VectorNd temV = new VectorNd(temR*temC);
      int k = 0;
      for (int i = 0; i < temR; i ++) {
         for (int j = 0; j < temC; j++) {
            temV.set (k, argV1.get (i)*argV2.get (j));
            k++;
         }
      }
      return temV;
   }

   /*
    * -----------------------------
    * private method
    * set D and N
    * -----------------------------
    */
   /**
    * set Dimension
    * @param argD
    */
   private void setDimesion(int argD) {
      myDimension = argD;
   }

   /**
    * set the Dimension of the column size of argP
    * @param argP N x D matrix
    */
   private void setDimesion(MatrixBase argP) {
      setDimesion(argP.colSize ());
   }

   /**
    * set the term number
    * @param argN
    */
   private void setN(int argN) {
      myN = argN;
   }

   /**
    * set the term number as the row size of argP
    * @param argP N x D matrix
    */
   private void setN(MatrixBase argP) {
      setN(argP.rowSize ());
   }

   /**
    * set the term number as the I*J*K...
    */
   public void setN() {
      int temNTotal = 1;
      for (int temN : myNList) {
         temNTotal = temNTotal * (temN+1);
      }
      setN(temNTotal);
   }

   //------------------------------------//
   /**
    * set the upper limits of DOF in each dimension
    * @param argNList
    */
   public void setNList(int [] argNList) {
      myNList = argNList;
      setDimesion(argNList.length);
      setN();
      myXSTU.setSize (myDimension);
   }

   /**
    * 
    * @return dimension
    */
   public int getDimension() {
      return myDimension;
   }

   /**
    * 
    * @return the term number
    */
   public int getN() {
      return myN;
   }

   public int [] getNList() {
      return myNList;
   }

   /**
    * set input X in STU coordinates
    * argX vector size should be myDimension
    * if the size of argX is not myDimension, it will be resize to myDimension
    * @param argX, [0, 1]
    */
   public void setXSTU(VectorNd argX) {
      if (argX.size () < myDimension) {
         for (int i = 0; i < argX.size (); i++) {
            myXSTU.set (i, argX.get (i));
         }
      }
      else {
         for (int i = 0; i < argX.size (); i++) {
            myXSTU.set (i, argX.get (i));
         }
      }
   }

   public VectorNd getXSTU() {
      return myXSTU;
   }

   /**
    * tensor product Bezier curve 
    * 
    * @return P^T * B
    */
   public VectorNd getTenProBezCur () {
      VectorNd tmpVec = new VectorNd ();
      tmpVec.mulTranspose (myP, myB); 
      return tmpVec;
   }

   /**
    * tensor product Bezier curve 
    * 
    * @return P^T * B
    */
   public VectorNd getTenProBezCur (double [] argX) {
      VectorNd tmpVec = new VectorNd();
      tmpVec.mulTranspose (myP, makeBlendingFunctions(argX)); 
      return tmpVec;
   }

   /**
    * tensor product Bezier curve 
    * 
    * @return P^T * B
    */
   public VectorNd getTenProBezCur (VectorNd argX) {
      VectorNd tmpVec = new VectorNd();
      tmpVec.mulTranspose (myP, makeBlendingFunctions(argX)); 
      return tmpVec;
   }


   /*
    * --------------------------------------------------
    * begin code testing
    * --------------------------------------------------
    */
   public void codeTest() {
      // test getBinomial
      int temN = 5;
      System.out.println("\nTest getBinomial(" + temN + ", i):");
      for (int i = 0; i <= temN; i ++) {
         System.out.print(getBinomial(temN,  i) + " ");
      }

      // test getBerBasis
      double temX = 0.5;
      System.out.println("\nTest getBerBasis(" + temN + ", i, " + temX + "):");
      for (int i = 0; i <= temN; i ++) {
         System.out.print(getBerBasis(temN,  i, temX) + " ");
      }

      // test setB 
      double [] temXArray = {0.7, 0.7, 0.7};
      setB(temXArray);
      System.out.println("\nTest getB(" + temXArray[0] + ", " + temXArray[1] + ", " + temXArray[2] + "):");
      PrintData.printVector (myB);

      // test setBba
      System.out.println("\nTest makeBlendingMatrix(" + temXArray[0] + ", " + temXArray[1] + ", " + temXArray[2] + "):");
      PrintData.printMatrix (getBMat());
   }

}
