package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Random;

import maspack.geometry.MeshBase;
import maspack.util.RandomGenerator;

/**
 * 
 * @author KeyiTang
 *
 */
public class UniformMeshFeatureSubsampler extends MeshInfoAllocator{

   
   protected MeshBase myTargetMesh;
   protected int myMax = Integer.MAX_VALUE;

   public UniformMeshFeatureSubsampler () {
      super();
   }

   public UniformMeshFeatureSubsampler (Collection<MeshBase> meshes) {
      super(meshes);
   }

   /**
    * 
    * @param maxV the maximum number of feature samples;
    */
   public void setMaxSampleNum(int maxV) {
      myMax = maxV;
   }

   public int getMaxSampleNum() {
      return myMax;
   }

   @Override
   public void allocateInfo (CloudInfo CI) throws 
   InstantiationException, IllegalAccessException {

      CI.clearInfoList ();

      int [] nums = new int [myMeshSet.size ()];
      int [] maxes = new int [myMeshSet.size ()];
      int totalNum = 0;
      ArrayList<MeshBase> meshes = new ArrayList<MeshBase> ();
      getMeshes(meshes);
      int idx = 0;
      if (myMeshFeature == MeshFeature.Vertex) {
         for (MeshBase mesh : meshes) {
            nums[idx] = mesh.getVertices ().size ();
            totalNum += nums[idx];
            idx++;
         }
         if (totalNum == 0) {
            System.out.println ("Warning: no vertex!");
            return;
         }
         for (int i = 0; i < nums.length; i++) {
            double num = (double)nums[i] / (double)totalNum * (double)myMax;
            if (num < 1) continue;
            maxes[i] = (int) Math.floor (num);
            assignVertex (CI, meshes.get (i), maxes[i]);
         }
      }
   }



   @Override
   public void addInfo (CloudInfo CI) throws 
   InstantiationException, IllegalAccessException {
      if (!CI.isAllocatable()) {
         throw new IllegalArgumentException (
         "Info container is not allocatable");
      }
      int max = myMax;
      int [] nums = new int [myMeshSet.size ()];
      int [] maxes = new int [myMeshSet.size ()];
      int totalNum = 0;
      ArrayList<MeshBase> meshes = new ArrayList<MeshBase> ();
      getMeshes(meshes);
      int idx = 0;
      if (myMeshFeature == MeshFeature.Vertex) {
         for (MeshBase mesh : meshes) {
            nums[idx] = mesh.getVertices ().size ();
            totalNum += nums[idx];
            idx++;
         }
         if (totalNum == 0) {
            System.out.println ("Warning: no vertex!");
            return;
         }
         for (int i = 0; i < nums.length; i++) {
            double num = nums[i] / totalNum * max;
            if (num < 1) continue;
            maxes[i] = (int) Math.floor (num);
            assignVertex (CI, meshes.get (i), maxes[i]);
         }
      }
   }


   /**
    * assumes that num is less that half the size of max
    * uniformly random
    * @param num
    * @param max
    * @return mark list
    */
   public static boolean[] createRandomBinaryIndices (int num, int max) {
      // assumes that num is less that half the size of max
      boolean[] marked = new boolean[max];
      int cnt = 0;
      Random randGen = RandomGenerator.get();
      if (num == 0) {
         return marked;
      }
      while (cnt < num) {
         int idx = randGen.nextInt (max-1);
         if (!marked[idx]) {
            marked[idx] = true;
            cnt++;
         }
      }
      return marked;
   }

   /**
    * assumes that num is less that half the size of max
    * uniformly random
    * @param num
    * @param max
    * @return mark list
    */
   public static int[] createRandomIndices (int num, int max) {
      boolean[] marked = new boolean[max];
      int [] indices = new int [num];
      int cnt = 0;
      Random randGen = RandomGenerator.get();
      while (cnt < num) {
         int idx = randGen.nextInt (max-1);
         if (!marked[idx]) {
            marked[idx] = true;
            indices[cnt] = idx;
            cnt++;
         }
      }
      return indices;
   }

   /**
    * Subsampling <tt>mesh</tt> vertices, make <code>VertexInfo</code> for each
    * sample vertex and add them into <code>info</code>;
    * @param info <code>CloudInfo</code> container
    * @param mesh mesh contains vertices to be sampled
    * @param max the maximum number of vertex samples
    * @throws IllegalAccessException 
    * @throws InstantiationException 
    */
   protected int assignVertex (CloudInfo vtxCI, MeshBase mesh, int maxv) throws 
   InstantiationException, IllegalAccessException {
      if (! VertexAllocatable.class.isAssignableFrom (vtxCI.getAllocatableInfoClass ())) {
         throw new IllegalArgumentException ("Incompatible cloud info container");
      }

      int numv = mesh.numVertices();
      int numd;

      if (numv <= maxv) {
         // just assign one ance per vertex
         numd = numv;
         for (int i=0; i<numd; i++) {
            VertexAllocatable info = (VertexAllocatable)vtxCI.getAllocatableInfoClass ().newInstance ();
            info.assignMesh (myTargetMesh);
            info.assignFeature (mesh.getVertex (i));
            info.setDirection (myDirection);
            vtxCI.addInfo ((ObjInfo)info);
         }
      }
      else {
         numd = maxv;
         // too many vertices; need to subsample
         System.out.println ("Too many vertices, subsampling mesh ...");
         if (maxv >= numv-maxv) {
            // faster to mark vertices for exclusion
            boolean[] marked = createRandomBinaryIndices (numv-maxv, numv);
            for (int i=0; i<marked.length; i++) {
               if (!marked[i]) {
                  VertexAllocatable info = (VertexAllocatable)vtxCI.getAllocatableInfoClass ().newInstance ();
                  info.assignMesh (myTargetMesh);
                  info.assignFeature (mesh.getVertex (i));
                  info.setDirection (myDirection);
                  vtxCI.addInfo ((ObjInfo)info);
               }
            }
         }
         else {
            // faster to mark vertices for inclusion
            boolean[] marked = createRandomBinaryIndices (maxv, numv);
            for (int i=0; i<marked.length; i++) {
               if (marked[i]) {
                  VertexAllocatable info = (VertexAllocatable)vtxCI.getAllocatableInfoClass ().newInstance ();
                  info.assignMesh (myTargetMesh);
                  info.assignFeature (mesh.getVertex (i));
                  info.setDirection (myDirection);
                  vtxCI.addInfo ((ObjInfo)info);
               }
            }
         }
      }
      System.out.println (numd + " vertices vertex infos have been allocated !");
      return numd;
   }

   private boolean myDirection = true;

   public void setDirection (boolean d) {
      myDirection = d;
   }

   public boolean getDirection () {
      return myDirection;
   }

   public void setTargetMesh (MeshBase mesh) {
      myTargetMesh = mesh;
   }
   
   public MeshBase getTargetMesh () {
      return myTargetMesh;
   }

   

}
