/*
 * Copyright 2012 Benjamin Glatzel <benjamin.glatzel@me.com>
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
package org.terasology.componentSystem.combat;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.vecmath.Vector3f;
import javax.xml.stream.events.EntityReference;

import org.terasology.componentSystem.UpdateSubscriberSystem;
import org.terasology.components.combat.HitDetectionComponent;
import org.terasology.components.world.LocationComponent;
import org.terasology.entitySystem.*;
import org.terasology.entitySystem.event.AddComponentEvent;
import org.terasology.entitySystem.event.RemovedComponentEvent;
import org.terasology.events.ActivateEvent;
import org.terasology.events.HitEvent;
import org.terasology.events.NoHealthEvent;
import org.terasology.game.CoreRegistry;
import org.terasology.math.QuatUtil;
import org.terasology.physics.CollideEvent;
import org.terasology.componentSystem.combat.HitDetection;
import org.terasology.componentSystem.combat.HitDetectionContext;
import org.terasology.componentSystem.combat.HitDetectionContextImpl;
import org.terasology.physics.PhysicsSystem;
import org.terasology.physics.RigidBodyComponent;
import org.terasology.physics.TriggerComponent;
import org.terasology.world.block.BlockComponent;

import com.bulletphysics.dynamics.RigidBody;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

/**
 * @author aherber 
 */
@RegisterComponentSystem
public class HitDetectionSystem implements EventHandlerSystem, UpdateSubscriberSystem {

	public HitDetectionContextImpl context;
	private volatile float delta = 0;
	@Override
	public void initialise() {
		context = new HitDetectionContextImpl();
        CoreRegistry.put(HitDetectionSystem.class, this);
	}

	@Override
	public void shutdown() {
	}
	
	/***
	 * @param event
	 * @param entity
	 * 
	 * 
	 */
	@ReceiveEvent(components = HitDetectionComponent.class,  priority = EventPriority.PRIORITY_NORMAL)
	public void onCollision(CollideEvent event, EntityRef entity) {
		EntityRef other = event.getOtherEntity();
		checkHitPosition(event);
		HitDetectionComponent hitDetection = entity.getComponent(HitDetectionComponent.class);
		if(hitDetection.hitBlocks || !other.hasComponent(org.terasology.world.block.BlockComponent.class)){
			if(hitDetection.trigger != HitDetection.DISABLED ){
				HitEvent hitEvent = new HitEvent(entity,other, event.getHitPoint(), event.getHitNormal());
				if(hitDetection.trigger == HitDetection.ONCE){//Collision Event doesnt need to be saved for later checks
					hitDetection.trigger = HitDetection.DISABLED;
					entity.saveComponent(hitDetection);
					entity.send(hitEvent);
				}else if(context.addHit(entity, other)){
					entity.send(hitEvent);
				}
			}
		}	
	}
	@ReceiveEvent(components = {HitDetectionComponent.class}, priority = EventPriority.PRIORITY_NORMAL)
	public void addHitDetection(AddComponentEvent event, EntityRef entity) {
	    context.getOncePerEntity().remove(entity);
	    context.getPeriodic().remove(entity);
	    context.getPeriodicPerEntity().remove(entity);  
	}
	
	@ReceiveEvent(components = {HitDetectionComponent.class})
	public void removeHitDetection(RemovedComponentEvent event, EntityRef entity) {
		HitDetectionComponent hitDetection = entity.getComponent(HitDetectionComponent.class);
	    switch (hitDetection.trigger) {
		case PERIODIC:
		    context.getPeriodic().remove(entity);
			break;
		case PERIODIC_PER_ENTITY:
		    context.getPeriodicPerEntity().remove(entity);
			break;
		case ONCE_PER_ENTITY:
			context.getOncePerEntity().remove(entity);
			break;
		default:
			break;
		}
	}
	
	/***
	 * @param event
	 * @param entity
	 * 
	 * 
	 */
	private synchronized void processHits(float delta){

		//Periodic
		HashMap<EntityRef,Float> removeMap = Maps.newHashMap();
		for(Entry<EntityRef,Float> entry:  context.getPeriodic().entrySet()){
			Float test =  entry.getValue();
			test = test-delta;
			entry.setValue(test);
			if(test <= 0){
				removeMap.put(entry.getKey(), test);
				entry = null;
			}
		}
		context.setPeriodic(Maps.newHashMap(Maps.difference(context.getPeriodic(),removeMap).entriesOnlyOnLeft()));
		//PeriodicPer Entity
		Set<Entry<EntityRef, HashMap<EntityRef, Float>>> entries = context.getPeriodicPerEntity().entrySet();
		for(Map.Entry<EntityRef,HashMap<EntityRef,Float>> entry : entries){
			Set<Entry<EntityRef, Float>> subEntries = entry.getValue().entrySet();
			if(entry != null && entry.getKey().exists()){
				for(Entry<EntityRef,Float> entryTemp :  subEntries){
					if(entryTemp != null && (entryTemp.getKey().exists())){
						Float test =  entryTemp.getValue();
						test = test-delta;
						entryTemp.setValue(test);
						if(test <= 0){
							entryTemp = null;//Eintrag entfernern //TODO this coud be a pro
						}
					}
				}
			}else{
				entry = null;//Eintrag entfernern
			}
		}
	}
	
	@Override
	public void update(float delta) {
		this.delta += delta;//handle lags
		float consume = this.delta;
		processHits(consume);
		this.delta-=consume;
	}
	
	 	/*
		 * 
		 */
		private float checkPositionRelativeToPlaneNormal(Vector3f delta,Vector3f direction, Vector3f planeNormal){
			float dotProduct;
			Vector3f cross = new Vector3f();
			cross.cross(direction,planeNormal);
			dotProduct = cross.dot(delta);
			return dotProduct;
		}
		
		private float checkPositionRelative(Vector3f delta,Vector3f direction){
			return direction.dot(delta);
		}
		
		private void checkHitPosition(CollideEvent event){
			EntityRef other = event.getOtherEntity();
			if( !other.hasComponent(BlockComponent.class)){
				Vector3f hitPoint = event.getHitPoint();
				Vector3f entityLocation = new Vector3f();
				Vector3f delta = new Vector3f();
				System.out.println("#############################################################");
				System.out.println("Check HitPosition:"+other.iterateComponents());
				System.out.println("CollideEvent:"+other);
				System.out.println("#############################################################");
				if(other.exists()){
					LocationComponent location = other.getComponent(LocationComponent.class);
					location.getWorldPosition(entityLocation);
					Vector3f direction = QuatUtil.getForwardVector(location.getWorldRotation());
//					System.out.println("fitpoint:"+hitPoint);
//					System.out.println("EntityLocation:"+entityLocation);
					delta.sub(entityLocation, hitPoint);
					//checkLeftRigth
					//float dotProduct = checkPositionRelativeToPlaneNormal(delta, direction, up);
					float dotProduct = checkPositionRelative(delta, direction);
					if(dotProduct < 0){
						System.out.println("Hitpoint is rigth to entityLocation");
						//do right hand stuff
					}else{
						System.out.println("Hitpoint is left to entityLocation");
						//do left hand stuff
					}
					//checkINfrontBehind
					//dotProduct = checkPositionRelativeToPlaneNormal(delta, direction, rigth);
					Vector3f rigth = QuatUtil.getRightVector(location.getWorldRotation()); 
					dotProduct = checkPositionRelative(delta, rigth);
					if(dotProduct < 0){
						System.out.println("Hitpoint is behind entityLocation");
						//do right hand stuff
					}else{
						System.out.println("Hitpoint is infront entityLocation");
						//do left hand stuff
					}
					//checkAboveBelow
					Vector3f up = QuatUtil.getUpVector(location.getWorldRotation()); 
					dotProduct = checkPositionRelativeToPlaneNormal(delta, direction, up);
					if(dotProduct < 0){
						System.out.println("Hitpoint is below entityLocation");
						//do right hand stuff
					}else{
						System.out.println("Hitpoint is above entityLocation");
						//do left hand stuff
					}
				}
			}
		}
}
