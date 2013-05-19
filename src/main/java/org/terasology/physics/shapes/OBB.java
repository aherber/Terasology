package org.terasology.physics.shapes;

import java.util.Iterator;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.terasology.math.QuatUtil;

import com.bulletphysics.linearmath.QuaternionUtil;

public class OBB {
	
	public Vector3f center = new Vector3f();
	public Vector3f extent = new Vector3f();
	public Vector3f xAxis = new Vector3f(1,0,0);
	public Vector3f yAxis = new Vector3f(0,1,0);
	public Vector3f zAxis = new Vector3f(0,0,1);
    private Logger logger = Logger.getLogger(getClass().getName());
    public String test;
	//test
	public void calculateFromPoints(String name, Vector3f pnts[]){
		Vector3f minX = null, maxX = null, minY = null, maxY = null, minZ = null, maxZ = null;
		Float minDotXAxis = null, maxDotXAxis = null, minDotYAxis = null, maxDotYAxis = null, minDotZAxis = null, maxDotZAxis = null;
		float curDot = 0;
		int numPoint = 0;
		if(pnts != null && pnts.length > 0){
			for(Vector3f curPoint : pnts){
				if(curPoint.x == 0 && curPoint.y == 0 && curPoint.z == 0)
					continue;
				if((curPoint.x <= 0.08 && curPoint.x >= -0.08) &&
				   (curPoint.y <= 0.08 && curPoint.y >= -0.08) &&
				   (curPoint.z <= 0.08 && curPoint.z >= -0.08)){
					continue; 
				} 
				center.add(curPoint);
				numPoint++;
				curDot = curPoint.dot(xAxis);
				if(minX == null || minDotXAxis == null || curDot <= minDotXAxis){
					minX = curPoint;		
					minDotXAxis = curDot;
				}
				if(maxX == null || maxDotXAxis == null || curDot >= maxDotXAxis){
					maxX = curPoint;
					maxDotXAxis = curDot;
				}
				curDot = curPoint.dot(yAxis);
				if(minY == null || minDotYAxis == null || curDot <= minDotYAxis){
					minY = curPoint;	
					minDotYAxis = curDot;
					
				}
				if(maxY == null || maxDotYAxis == null || curDot >= maxDotYAxis){
					maxY = curPoint;
					maxDotYAxis = curDot;
				}
				curDot = curPoint.dot(zAxis);
				if(minZ == null || minDotZAxis == null || curDot <= minDotZAxis){
					minZ = curPoint;	
					minDotZAxis = curDot;
				}
				if(maxZ == null || maxDotZAxis == null || curDot >= maxDotZAxis){
					maxZ = curPoint;
					maxDotZAxis = curDot;
				}
			}
			if(minX != null && maxX != null && 
			   minY != null && maxY != null && 
			   minZ != null && maxZ != null){

				float x = maxX.x - minX.x;
				float y = maxY.y - minY.y;
				float z = maxZ.z - minZ.z;
				extent.x = x != 0 ? x/2 : x; 
				extent.y = y != 0 ? y/2 : y; 
				extent.z = z != 0 ? z/2 : z;
				
				center.x = minX.x+(x/2);
				center.y = minY.y+(y/2);
				center.z = minZ.z+(z/2);
				
			}
		}
	}
	
	public void calculateFromPoints(String name, Vector3f pnts[], Quat4f rotation){
		Vector3f minX = null, maxX = null, minY = null, maxY = null, minZ = null, maxZ = null;
		Float minDotXAxis = null, maxDotXAxis = null, minDotYAxis = null, maxDotYAxis = null, minDotZAxis = null, maxDotZAxis = null;
		Vector3f axis = new Vector3f(1,1,1);
        QuaternionUtil.quatRotate(rotation, axis, axis);
		Vector3f xAxis = QuatUtil.getRightVector(rotation);
		Vector3f yAxis = QuatUtil.getUpVector(rotation);
		Vector3f zAxis = QuatUtil.getForwardVector(rotation); 
		float curDot = 0;
		int numPoint = 0;
		if(pnts != null && pnts.length > 0){
			for(Vector3f curPoint : pnts){
				if(curPoint.x == 0 && curPoint.y == 0 && curPoint.z == 0)
					continue;
				if((curPoint.x <= 0.05 && curPoint.x >= -0.05) &&
				   (curPoint.y <= 0.05 && curPoint.y >= -0.05) &&
				   (curPoint.z <= 0.05 && curPoint.z >= -0.05)){
					continue; 
				} 
				center.add(curPoint);
				numPoint++;
				curDot = curPoint.dot(xAxis);
				if(minX == null || minDotXAxis == null || curDot <= minDotXAxis){
					minX = curPoint;		
					minDotXAxis = curDot;
				}
				if(maxX == null || maxDotXAxis == null || curDot >= maxDotXAxis){
					maxX = curPoint;
					maxDotXAxis = curDot;
				}
				curDot = curPoint.dot(yAxis);
				if(minY == null || minDotYAxis == null || curDot <= minDotYAxis){
					minY = curPoint;	
					minDotYAxis = curDot;
				}
				if(maxY == null || maxDotYAxis == null || curDot >= maxDotYAxis){
					maxY = curPoint;
					maxDotYAxis = curDot;
				}
				curDot = curPoint.dot(zAxis);
				if(minZ == null || minDotZAxis == null || curDot <= minDotZAxis){
					minZ = curPoint;	
					minDotZAxis = curDot;
				}
				if(maxZ == null || maxDotZAxis == null || curDot >= maxDotZAxis){
					maxZ = curPoint;
					maxDotZAxis = curDot;
				}
			}
			if(minX != null && maxX != null && 
			   minY != null && maxY != null && 
			   minZ != null && maxZ != null){

				float x = maxX.x - minX.x;
				float y = maxY.y - minY.y;
				float z = maxZ.z - minZ.z;
				
				extent.x = x != 0 ? x/2 : x; 
				extent.y = y != 0 ? y/2 : y; 
				extent.z = z != 0 ? z/2 : z;
				
				center.x = minX.x+(x/2);
				center.y = minY.y+(y/2);
				center.z = minZ.z+(z/2);
			}
		}
	}
}
