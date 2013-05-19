/*
 * Copyright 2012  Benjamin Glatzel <benjamin.glatzel@me.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.terasology.componentSystem;

import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.newdawn.slick.util.Log;
import org.terasology.componentSystem.RenderSystem;
import org.terasology.componentSystem.UpdateSubscriberSystem;
import org.terasology.physics.CollideEvent;
import org.terasology.physics.CollisionGroup;
import org.terasology.physics.PhysicsSystem;
import org.terasology.physics.StandardCollisionGroup;
import org.terasology.physics.TriggerComponent;
import org.terasology.physics.shapes.BoxShapeComponent;
import org.terasology.physics.shapes.OBB;
import org.terasology.rendering.AABBRenderer;
import org.terasology.rendering.assets.animation.MeshAnimation;
import org.terasology.rendering.logic.AnimEndEvent;
import org.terasology.rendering.logic.SkeletalMeshComponent;
import org.terasology.components.HitAreaComponent;
import org.terasology.components.world.LocationComponent;
import org.terasology.entitySystem.EntityManager;
import org.terasology.entitySystem.EntityRef;
import org.terasology.entitySystem.EventHandlerSystem;
import org.terasology.entitySystem.EventPriority;
import org.terasology.entitySystem.ReceiveEvent;
import org.terasology.entitySystem.RegisterComponentSystem;
import org.terasology.entitySystem.event.AddComponentEvent;
import org.terasology.game.CoreRegistry;
import org.terasology.logic.LocalPlayer;
import org.terasology.logic.manager.ShaderManager;
import org.terasology.math.AABB;
import org.terasology.math.QuatUtil;
import org.terasology.math.Vector3fUtil;
import org.terasology.rendering.assets.animation.MeshAnimationFrame;
import org.terasology.rendering.assets.skeletalmesh.Bone;
import org.terasology.rendering.world.WorldRenderer;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.lwjgl.opengl.GL11.GL_LINE_LOOP;
import static org.lwjgl.opengl.GL11.GL_LINES;
import static org.lwjgl.opengl.GL11.GL_QUADS;
import static org.lwjgl.opengl.GL11.glBegin;
import static org.lwjgl.opengl.GL11.glCallList;
import static org.lwjgl.opengl.GL11.glEnd;
import static org.lwjgl.opengl.GL11.glEndList;
import static org.lwjgl.opengl.GL11.glGenLists;
import static org.lwjgl.opengl.GL11.glMultMatrix;
import static org.lwjgl.opengl.GL11.glNewList;
import static org.lwjgl.opengl.GL11.glPopMatrix;
import static org.lwjgl.opengl.GL11.glPushMatrix;
import static org.lwjgl.opengl.GL11.glTranslated;
import static org.lwjgl.opengl.GL11.glVertex3f;
import static org.lwjgl.opengl.GL11.glColor4f;
import static org.lwjgl.opengl.GL11.glDisable;
import static org.lwjgl.opengl.GL11.glLineWidth;
import static org.lwjgl.opengl.GL11.glEnable;

/**
 * @author aherber
 */
@RegisterComponentSystem
public class HitAreaSystem implements RenderSystem, EventHandlerSystem, UpdateSubscriberSystem {

    private Logger logger = Logger.getLogger(getClass().getName());
    private EntityManager entityManager;
    private AABBRenderer aabbRenderer=new AABBRenderer(null);

    @Override
    public void initialise() {
        entityManager = CoreRegistry.get(EntityManager.class);
    }

    @Override
    public void shutdown() {
    }

  @ReceiveEvent(components = {SkeletalMeshComponent.class, LocationComponent.class})
  public void newHitArea(AddComponentEvent event, EntityRef entity) {
	    SkeletalMeshComponent skeleton = entity.getComponent(SkeletalMeshComponent.class);
	    if (skeleton.mesh == null) {
	        System.out.println("Mesh is null !!!"+skeleton.mesh);
	        return;
	    }
	    HitAreaComponent hitArea = new HitAreaComponent();
	    hitArea.hitAreaEntities = Maps.newHashMap(); 
	    calculate(entity,skeleton.mesh.getRootBone(),skeleton,hitArea);
	    hitArea.rootBone = hitArea.hitAreaEntities.get(skeleton.mesh.getRootBone().getName());
	    entity.saveComponent(hitArea);
	    hookToSkeleton(entity,skeleton, hitArea.hitAreaEntities );
  }
  
  public void hookToSkeleton(EntityRef entity, SkeletalMeshComponent skeleton, Map<String, EntityRef> hitAreas){
	  	Map<String,Vector3f> translations = Maps.newHashMap(); 
	    for(Bone curBone : skeleton.mesh.bones()){
	    	logger.log(Level.INFO, "bone:"+curBone.getName());
	    	EntityRef boneEntity = skeleton.boneEntities.get(curBone.getName()); 
	    	LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
	    	EntityRef entityHitArea = hitAreas.get(curBone.getName());
	    	LocationComponent locHit = entityHitArea.getComponent(LocationComponent.class);
	    	Vector3f translation = new Vector3f();
	        Vector3f worldPositionBone = loc.getWorldPosition();
	    	Vector3f worldPositionHitArea = locHit.getWorldPosition();
//	    	logger.log(Level.INFO, "worldPositionHitArea:"+worldPositionHitArea);
//	    	logger.log(Level.INFO, "worldPositionBone:"+worldPositionBone);
//	    	logger.log(Level.INFO, "worldPositionHitArea.length():"+worldPositionHitArea.length());
//	    	logger.log(Level.INFO, "worldPositionBone.length():"+worldPositionBone.length());
//	    	logger.log(Level.INFO, "locHit.getWorldRotation():"+locHit.getWorldRotation());
//	    	logger.log(Level.INFO, "loc.getWorldRotation():"+loc.getWorldRotation());
//	    	logger.log(Level.INFO, "locHit.getLocalRotation():"+locHit.getLocalRotation());
//	    	logger.log(Level.INFO, "loc.getLocalRotation():"+loc.getLocalRotation());
	//    	if(worldPositionBone.length() >= worldPositionHitArea.length()){
	//	    	translation.sub(worldPositionBone, worldPositionHitArea);
	//    	}else{
	    		translation.sub(worldPositionHitArea, worldPositionBone);
	//    	}
	    	translations.put(curBone.getName(), translation);
	    }
	    LocationComponent entityLoc = entity.getComponent(LocationComponent.class);
	    Quat4f worldRot = entityLoc.getWorldRotation(); 
	    Quat4f inverseWorldRot = new Quat4f();
	    inverseWorldRot.inverse(worldRot);
	    for(Bone curBone : skeleton.mesh.bones()){
	    	Vector3f translation = translations.get(curBone.getName());
	    	EntityRef boneEntity = skeleton.boneEntities.get(curBone.getName()); 
	    	LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
	    	EntityRef entityHitArea = hitAreas.get(curBone.getName());
	    	LocationComponent locHit = entityHitArea.getComponent(LocationComponent.class);
            QuaternionUtil.quatRotate(inverseWorldRot, translation, translation);
            Quat4f rot = new Quat4f();
            rot.mul(inverseWorldRot, loc.getWorldRotation());
            QuaternionUtil.quatRotate(rot, translation, translation);
            locHit.setLocalRotation(rot);
	    	locHit.setLocalPosition(translation);
	    	entityHitArea.saveComponent(locHit);
	    	loc.addChild(entityHitArea, boneEntity);
	    	boneEntity.saveComponent(loc); 
	    } 
  }
        
  public void calculate(EntityRef entity, Bone bone, SkeletalMeshComponent skeleton, HitAreaComponent hitArea){
		LocationComponent loc = new LocationComponent();
	    loc.setLocalPosition(bone.getLocalPosition());
	   // loc.setLocalRotation(bone.getLocalRotation());
		BoxShapeComponent box = null;
	    List<Vector3f> boneVertices = skeleton.mesh.getVertexPositionsBone(bone);
	    OBB obb = new OBB();
	    Vector3f[] pnts = new Vector3f[boneVertices.size()];
	    boneVertices.toArray(pnts);
	    obb.calculateFromPoints(bone.getName(),pnts);
//	    obb.calculateFromPoints(bone.getName(), pnts, bone.getLocalRotation());
	//	 loc.setLocalRotation(bone.getLocalRotation());

    	logger.log(Level.INFO, "bone:"+bone.getName());
    	Vector3f pos = new Vector3f();
 		box = new BoxShapeComponent();
 		box.extents.set(new Vector3f(obb.extent.x, obb.extent.y, obb.extent.z));
 		loc.setLocalPosition(obb.center);
 		EntityRef hitAreaEntity = entityManager.create(loc);
 		EntityRef parent = (bone.getParent() != null && hitArea.hitAreaEntities.get(bone.getParent().getName()) != null) ? hitArea.hitAreaEntities.get(bone.getParent().getName()) : entity;
 		if (parent != entity){
 			LocationComponent parentLoc = parent.getComponent(LocationComponent.class);
 			parentLoc.addChild(hitAreaEntity, parent);
 			parent.saveComponent(parentLoc);
   	 		while(parent != entity && parent != null){
   	   	 		parentLoc = parent.getComponent(LocationComponent.class);
   	   	 		pos.add(parentLoc.getLocalPosition());
   	   	 		parent = parentLoc.getParent();
   	 		}
   	 		pos.sub(loc.getLocalPosition(),pos);
   	 		loc.setLocalPosition(pos);
        }else{
        	LocationComponent parentLoc = parent.getComponent(LocationComponent.class);
        	parentLoc.addChild(hitAreaEntity, parent);
        	parent.saveComponent(parentLoc);
        }
        hitAreaEntity.saveComponent(loc);
        hitArea.hitAreaEntities.put(bone.getName(), hitAreaEntity);
        if(box != null){
        	hitAreaEntity.addComponent(box);
        	TriggerComponent trigger = new TriggerComponent();
        	//TODO this should be configurable 
            trigger.detectGroups.add(StandardCollisionGroup.CHARACTER);
            trigger.detectGroups.add(StandardCollisionGroup.WORLD);
            trigger.detectGroups.add(StandardCollisionGroup.SENSOR);
            hitAreaEntity.addComponent(trigger);
        }
 	    for(Bone child : bone.getChildren()){
	      	calculate(entity,child,skeleton,hitArea);
	  	}
   }


    @Override
    public void update(float delta) {
    }

    @Override
    public void renderOpaque() {
    }

    @Override
    public void renderTransparent() {
    }

    @Override
    public void renderOverlay() {
//        ShaderManager.getInstance().enableDefault();
//        Vector3f cameraPosition = worldRenderer.getActiveCamera().getPosition();
//        glPushMatrix();
//        glDisable(GL11.GL_TEXTURE_2D);
//        glLineWidth(2);
//        glTranslated(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
//        glColor4f(1.0f, 0.0f, 1.0f, 1.0f);
    	 for (EntityRef entity : entityManager.iteratorEntities(HitAreaComponent.class, LocationComponent.class)) {
             HitAreaComponent skeletalMesh = entity.getComponent(HitAreaComponent.class);
             renderBoneCollisionVolumes(skeletalMesh.hitAreaEntities);
             //renderBone(skeletalMesh.rootBone);
         }
//         glEnable(GL11.GL_TEXTURE_2D);
//         glPopMatrix();
    }
    
    private void renderBone(EntityRef boneEntity) {
        LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
        if (loc == null) {
            return;
        }
        LocationComponent parentLoc = loc.getParent().getComponent(LocationComponent.class);
        if (parentLoc != null) {
            glPushMatrix();
            Vector3f worldPosA = loc.getWorldPosition();
            Vector3f worldPosB = parentLoc.getWorldPosition();
            glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
            glBegin(GL11.GL_LINES);
            glVertex3f(worldPosA.x, worldPosA.y, worldPosA.z);
            glVertex3f(worldPosB.x, worldPosB.y, worldPosB.z);
            glEnd();
            for (EntityRef child : loc.getChildren()) {
                renderBone(child);
            }
            glPopMatrix();
        }
    }
    
    private void renderBoneCollisionVolumes(Map<String,EntityRef>boneEntities) {
    	for(EntityRef boneEntity : boneEntities.values()){
    		LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
    		BoxShapeComponent box = boneEntity.getComponent(BoxShapeComponent.class);
    		if (loc == null || box ==  null) {
    			continue;
    		}
    		Quat4f worldRot = loc.getWorldRotation();
    		Vector3f dimensions = new Vector3f(box.extents);
    		AABB aabb = AABB.createCenterExtent(loc.getWorldPosition(), dimensions);
    		aabbRenderer.setAABB(aabb);
    	    aabbRenderer.render(2f, worldRot);
    	}
    }
    
    private void renderBoneCollisionVolumes(EntityRef boneEntity) {
    		LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
    		BoxShapeComponent box = boneEntity.getComponent(BoxShapeComponent.class);
    		if (loc == null || box ==  null) {
    			 return;
    		}
    		Quat4f worldRot = loc.getWorldRotation();
    		Vector3f dimensions = new Vector3f(box.extents);
    		AABB aabb = AABB.createCenterExtent(loc.getWorldPosition(), dimensions);
    		aabbRenderer.setAABB(aabb);
    	    aabbRenderer.render(2f, worldRot);
    		for(EntityRef child : loc.getChildren()){
        	    renderBoneCollisionVolumes(child);
    		}
    }
    
    private void renderPosition(EntityRef boneEntity) {
        LocationComponent loc = boneEntity.getComponent(LocationComponent.class);
        if (loc == null) {
            return;
        }
        LocationComponent parentLoc = loc.getParent().getComponent(LocationComponent.class);
        if (parentLoc != null) {
            glPushMatrix();
            Vector3f worldPosA = loc.getWorldPosition();
            Vector3f worldPosB = parentLoc.getWorldPosition();

            glBegin(GL11.GL_LINES);
            glVertex3f(worldPosA.x, worldPosA.y, worldPosA.z);
            glVertex3f(worldPosB.x, worldPosB.y, worldPosB.z);
            glEnd();

            for (EntityRef child : loc.getChildren()) {
            	renderPosition(child);
            }
            glPopMatrix();
        }
    }
    
    
    @Override
    public void renderFirstPerson() {
    }

}
