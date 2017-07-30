package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.util.ArrayList;
import java.util.List;

public class CloudInfo <T extends ObjInfo> extends ObjInfo{
   
   ArrayList<T> myInfoList =  new ArrayList<T> ();
   protected boolean isAllocatable;
   
   public CloudInfo () {
      isAllocatable = false;
   }
   
   public boolean isAllocatable () {
      return isAllocatable;
   }
   
   public void getInfos(List<T> list) {
      list.addAll (myInfoList);
   }
   
   public List<T> getInfos () {
      return myInfoList;
   }

   @Override
   /**
    * This will update all sub-infos
    */
   public boolean update() {
      boolean flag = true;
      for (T info : myInfoList) {
         if (flag == true) {
            flag = info.update ();
         } else {
            info.update ();
         }
      }
      
      if (flag = true) {
         return super.update ();
      } else {
         super.update ();
         return false;
      }
   }
   
   
   public boolean addInfo (T info) {
      if (myInfoList.add (info)) {
         return true;
      } else {
         return false;
      }
   }

   public T getInfo(int index) {
      return myInfoList.get (index);
   }
   
   public T getInfo (String infoName) {
      for (T info : myInfoList) {
         if (info.getName ().equalsIgnoreCase ("infoName")) {
            return info;
         }
      }
      return null;
   }
   
   public boolean removeInfo(T info) {
      if (myInfoList.remove (info)) {
         return true;
      } else {
         return false;
      }
   }
   
   public void clearInfoList () {
      for (T info : myInfoList) {
         removeInfo(info);
      }
   }
   
   public int numInfos () {
      return myInfoList.size ();
   }
   
   public Class<?> getAllocatableInfoClass () {
      return null;
   }
}
