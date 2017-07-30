package artisynth.models.swallowingRegistrationTool.utilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.subjectFrank.SubjectModel;
import artisynth.models.subjectFrank.subjects.SwallowPatient;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.VectorNd;

public class RegistrationReporter {
   
   

   public static void main (String [] args) {
      String path1 = ArtisynthPath.getSrcRelativePath (SwallowPatient.class, "swallow_CT04mm/");
      String path2 = ArtisynthPath.getSrcRelativePath (SwallowPatient.class, "speech_MRI/");
      String path3 = "/Users/KeyiTang/Dropbox/Experiments/ModelRegistration/";
      String path4 = "/Users/KeyiTang/Dropbox/Experiments/";
      String path5 = ArtisynthPath.getSrcRelativePath (SwallowPatient.class, "speech_subject3/");
      
      /*
      writeFEM_ANSYS (
        "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elements/subject1_tongue",
        "/Users/KeyiTang/Dropbox/Experiments/MRI/tonguePharynx/subject1_tongue");*/
      
      //writeFEM_ANSYS (
         //path1 + "registration/face/faceSourceFem",
        // "/Users/KeyiTang/Dropbox/Experiments/face/faceSourceFem");
      
      //convertANSYS2VTK (
         //"/Users/KeyiTang/Dropbox/Experiments/larynx/larynxRepairedFem",
         //"/Users/KeyiTang/Dropbox/Experiments/larynx/larynxRepairedFem");
      
      /*
      convertANSYS2VTK (
         "/Users/KeyiTang/Dropbox/Experiments/AAC/tongueRepaired",
         "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elements/subject1_tongueRepaired");
      
     
     writeFEMQualities (
         "/Users/KeyiTang/Dropbox/Experiments/CT/tonguePharynx/tongueRepairedFem",
         "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/expHybridTongueCT_MMRep.txt");*/
     
      /*
     writeFEMQualities (
        path5 + "tongue/tongueFem",
       "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speaker3/tongueQuality.txt");
     
     writeFEMQualities (
        path5 + "pharynx/pharynxFem",
       "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speaker3/pharynxQuality.txt");
     
     writeFEMQualities (
        path5 + "softPalate/softPalateFem",
       "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speaker3/softPalateQuality.txt");
     
     writeFEMQualities (
        path5 + "larynx/larynxFem",
       "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speaker3/larynxQuality.txt");
     
     writeFEMQualities (
        path5 + "face/faceFem",
       "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speaker3/faceQuality.txt");*/

     /*
     printFEMQualities (
       "/Users/KeyiTang/Documents/softPalateFem");
     printFEMQualities (
     "/Users/KeyiTang/Documents/tongueFem");
     printFEMQualities (
     "/Users/KeyiTang/Documents/faceFem");*/
      
     /*
      writeFEMQualities (
      path4 + "softPalate/softPalateFem",
      "/Users/KeyiTang/Documents/MATLAB/SubjectModel/data/elementQualities/speechSoftPalate.txt");*/
      
      
      Map <PolygonalMesh, PolygonalMesh> meshes = new LinkedHashMap <PolygonalMesh, PolygonalMesh> ();
      
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/jaw/jawPartialResultMesh"), 
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/jaw/jawPartialTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/upperLip/upperLipForFaceResultMesh1"), 
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/upperLip/upperLipTargetMesh"));
 
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/maxilla/maxillaPartialResultMesh"), 
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/maxilla/maxillaPartialResultMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/tongue/tongueResultMesh"),
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/tongue/tongueTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/airway/airwayPartialResultMesh"),
         SubjectModel.readMeshWithoutSuffix (
            path5 + "registration/airway/airwayTargetMesh"));
      
      //meshes.put (
         //SubjectModel.readMeshWithoutSuffix (
           // path1 + "registration/softPalate/softPalateResultMesh2"), 
        // SubjectModel.readMeshWithoutSuffix (
          //  path1 + "registration/softPalate/softPalateTargetMesh"));
      
     // meshes.put (
       //  SubjectModel.readMeshWithoutSuffix (
           // path1 + "registration/upperLarynx/upperLarynxResultMesh"), 
      //   SubjectModel.readMeshWithoutSuffix (
         //   path1 + "registration/upperLarynx/upperLarynxTargetMesh"));
      
      
      /*
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path3 + "CT1/softPalateResultMesh"), 
         SubjectModel.readMeshWithoutSuffix (
            path1 + "registration/softPalate/softPalateTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path4 + "CT/pharynx/airwayForPharynxResultMesh_AT"),
         SubjectModel.readMeshWithoutSuffix (
            path4 + "CT/pharynx/airwayForPharynxTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path4 + "CT/pharynx/airwayForPharynxResultMesh_MMRep1"),
         SubjectModel.readMeshWithoutSuffix (
            path4 + "CT/pharynx/airwayForPharynxTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path4 + "MRI/pharynx1/airwayForPharynxResultMesh_AT"),
         SubjectModel.readMeshWithoutSuffix (
            path4 + "MRI/pharynx1/airwayForPharynxTargetMesh"));
      
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path4 + "MRI/pharynx1/airwayForPharynxResultMesh_MMRep"),
         SubjectModel.readMeshWithoutSuffix (
            path4 + "MRI/pharynx1/airwayForPharynxTargetMesh"));*/
      
      
      /*
      PolygonalMesh mesh = SubjectModel.readMeshWithoutSuffix (
         path2 + "geometry/airway/airwaySubjectMesh");
      mesh.addMesh (SubjectModel.readMeshWithoutSuffix (
         path2 + "registration/airwayForSoftPalate/airwayForSoftPalateTargetMesh"));
      meshes.put (
         SubjectModel.readMeshWithoutSuffix (
            path2 + "geometry/airway/airwayResultMesh"),
         mesh);*/
      
      Set ens = meshes.entrySet ();
      Iterator it = ens.iterator ();
      int i = 0;
      int num = 0;
      ArrayList<RegistrationReport> reports = new ArrayList<RegistrationReport> ();
      while (it.hasNext ()) {
         Entry <PolygonalMesh, PolygonalMesh> me = (Entry <PolygonalMesh, PolygonalMesh>)it.next ();
         RegistrationReport report = RegistrationReporter.reportSurfaceDistance (me.getKey (), me.getValue (), i++);
         num += report.src2tgtErrs.length;
         System.out.println ("mean: " + report.ErrSrc2Tgt);
         System.out.println ("dev: " + report.DevSrc2Tgt);
         VectorNd errs = new VectorNd (report.src2tgtErrs);
         System.out.println ("max:" + errs.maxElement ());
         System.out.println ("");
         reports.add (report);
      }
      
      RegistrationReport reportSum = new RegistrationReport (100);
      reportSum.src2tgtErrs = new double [num];
      i = 0;
      for (RegistrationReport report : reports) {
         for (double err : report.src2tgtErrs) {
            reportSum.src2tgtErrs[i++] = err;
         }
      }
      reportSum.computeDevSrc2Tgt ();
      System.out.println ("mean: " + reportSum.ErrSrc2Tgt);
      System.out.println ("dev: " + reportSum.DevSrc2Tgt);
      VectorNd errs = new VectorNd (reportSum.src2tgtErrs);
      System.out.println ("max:" + errs.maxElement ());
      
   }
   
   public static void writeFEM_ANSYS (String femPath, String fileName) {
      FemModel3d fem = new FemModel3d ();
      SubjectModel.readFemWithoutSuffix (fem, femPath);
      
      FemModelAgent agent = new FemModelAgent (fem);
      try {
         ReadWrite.writeNodesToAnsys (agent, fileName+".node");
         ReadWrite.writeElementToAnsys (agent, fileName+".elem");
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      
   }
   
   public static void convertANSYS2VTK (String femPathWithoutSuffix, String fileName) {
      FemModel3d fem = new FemModel3d ();
      try {
         AnsysReader.read (fem, femPathWithoutSuffix + ".node", femPathWithoutSuffix + ".elem");
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      
      FemModelAgent agent = new FemModelAgent (fem);
      SubjectModel.writeFemWithoutSuffix (fem, fileName);
   }
   
   public static void writeFEMQualities (String femPathWithoutSuffix, String fileName) {
      FemModel3d fem = new FemModel3d ();
      SubjectModel.readFemWithoutSuffix (fem, femPathWithoutSuffix);
      
      try {
         FEMQualityPrinter.writeMeanRatios (
            fem, fileName);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
   
   public static void printFEMQualities (String femPathWithoutSuffix) {
      FemModel3d fem = new FemModel3d ();
      SubjectModel.readFemWithoutSuffix (fem, femPathWithoutSuffix);
      fem.resetRestPosition ();
      System.out.println (
         "Artisynth: number of inverted rest elements: " 
      + fem.markInvertedRestElements ());

      //FEMQualityPrinter.printMeanRatios (fem);

   }

   public static RegistrationReport reportSurfaceDistance (
      PolygonalMesh srcMesh, PolygonalMesh tgtMesh, int index) {
      RegistrationReport report = new RegistrationReport (index);
      
      int numSrc = srcMesh.numVertices ();
      int numTgt = tgtMesh.numVertices ();
      report.src2tgtErrs = new double [numSrc];
      report.tgt2srcErrs = new double [numTgt];
      
      BVFeatureQuery bv = new BVFeatureQuery ();
      double sum = 0;
      for (int i = 0; i < numSrc; i++) {
         Vertex3d vtx = srcMesh.getVertex (i);
         Point3d pnt = new Point3d ();
         vtx.getWorldPoint (pnt);
         Point3d near = new Point3d ();
         Vector2d uv = new Vector2d ();
         bv.nearestFaceToPoint (near, uv, tgtMesh, pnt);
         report.src2tgtErrs[i] = near.distance (pnt);
         sum += report.src2tgtErrs[i];
      }
      report.ErrSrc2Tgt = sum / numSrc;
      
      double sumDual = sum;
      sum = 0;
      for (int i = 0; i < numTgt; i++) {
         Vertex3d vtx = tgtMesh.getVertex (i);
         Point3d pnt = new Point3d ();
         vtx.getWorldPoint (pnt);
         Point3d near = new Point3d ();
         Vector2d uv = new Vector2d ();
         bv.nearestFaceToPoint (near, uv, srcMesh, pnt);
         report.tgt2srcErrs[i] = near.distance (pnt);
         sum += report.tgt2srcErrs[i];
      }
      report.ErrTgt2Src = sum / numTgt;
      sumDual += sum;
      report.ErrDual = sumDual / (numTgt + numSrc);
      
      double DevSum = 0;
      for (int i = 0; i < numSrc; i++) {
         DevSum += Math.pow ((report.src2tgtErrs [i] - report.ErrSrc2Tgt), 2);
      }
      double dev = Math.sqrt (DevSum);
      dev = dev / Math.sqrt (numSrc);
      report.DevSrc2Tgt = dev;
      
      double DevSumDual = DevSum;
      DevSum = 0;
      for (int i = 0; i < numSrc; i++) {
         DevSum += Math.pow ((report.tgt2srcErrs [i] - report.ErrTgt2Src), 2);
      }
      dev = Math.sqrt (DevSum);
      dev = dev / Math.sqrt (numTgt);
      report.DevTgt2Src = dev;
      
      DevSumDual += DevSum;
      dev = Math.sqrt (DevSumDual);
      dev = dev / Math.sqrt (numSrc + numTgt);
      report.DevDual = dev;
      
      return report;
   }
}


