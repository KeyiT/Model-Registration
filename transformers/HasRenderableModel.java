package artisynth.models.swallowingRegistrationTool.transformers;

import maspack.render.Renderable;

public interface HasRenderableModel {
   /**
    * This method would be called implicitly by registration manager
    * for rendering. 
    * @return renderable model; if no model for rendering return null
    */
   public abstract Renderable getTransformerRenderer();
}
