package main.java.org.fog.placement;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import main.java.org.fog.mobilitydata.Location;
import main.java.org.fog.mobilitydata.DataParser;
import main.java.org.fog.mobilitydata.References;
import main.java.org.fog.utils.Config;

public class LocationHandler {
	
	public DataParser dataObject;
	public Map<Integer, String> instanceToDataId;
	

	public LocationHandler(DataParser dataObject) {
		// TODO Auto-generated constructor stub
		this.dataObject = dataObject;
		instanceToDataId = new HashMap<Integer, String>();
		
	}

	public LocationHandler() {
		// TODO Auto-generated constructor stub

	}
	
	public DataParser getDataObject(){
		return dataObject;
	}
	
	public static double calculateDistance(Location loc1, Location loc2) {

	    final int R = 6371; // Radius of the earth in Kilometers

	    double latDistance = Math.toRadians(loc1.latitude - loc2.latitude);
	    double lonDistance = Math.toRadians(loc1.longitude - loc2.longitude);
	    double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
	            + Math.cos(Math.toRadians(loc1.latitude)) * Math.cos(Math.toRadians(loc2.latitude))
	            * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
	    double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
	    double distance = R * c; // kms


	    distance = Math.pow(distance, 2);

	    return Math.sqrt(distance);
	}
	
	
	/**
	 * Checks if a child node is within the maximum communication range of its parent node.
	 */
	public Boolean IsNodeInParentRange(int resourceId, int parentResourceId, double time) {
		if(time == References.INIT_TIME) {
			return false;
		}
		
		String dataId = getDataIdByInstanceID(resourceId);
		int resourceLevel=getDataObject().resourceAndUserToLevel.get(dataId);
		int parentLevel = resourceLevel-1;
		if(resourceLevel == getDataObject().levelID.get("Mobile")) {
			parentLevel = 1;
		}
		//Skip distance check for 0 or -1 parents
		if(parentLevel == 0 || parentLevel == -1) {
			return true;
		}
		
		Location resourceLoc;
		if(resourceLevel!=getDataObject().levelID.get("User") && resourceLevel!=getDataObject().levelID.get("Mobile"))
			resourceLoc = getResourceLocationInfo(dataId);
		else
			resourceLoc = getUserLocationInfo(dataId,time);
		
		String parentDataId = getDataIdByInstanceID(parentResourceId);
		int parentResourceLevel=getDataObject().resourceAndUserToLevel.get(dataId);
		Location parentResourceLoc = null;
		
		try {
			if(parentResourceLevel!=getDataObject().levelID.get("User") && parentResourceLevel!=getDataObject().levelID.get("Mobile"))
				parentResourceLoc = getResourceLocationInfo(parentDataId);
			else
				parentResourceLoc = getUserLocationInfo(parentDataId,time);

		}
		catch (Exception e) {
			System.out.println(e.getMessage());
		}
		
		if(parentResourceLoc == null) {
			return false;
		}
		
	    double distance = calculateDistance(resourceLoc, parentResourceLoc);
	    
	    return distance < (Config.MIN_COMMUNICATION_RANGE/1000);
	}

	public int determineParent(int resourceId, double time) {
		// TODO Auto-generated method stub
		String dataId = getDataIdByInstanceID(resourceId);
		int resourceLevel=getDataObject().resourceAndUserToLevel.get(dataId);
		int parentLevel = resourceLevel-1;
		if(resourceLevel == getDataObject().levelID.get("Mobile")) {
			parentLevel = 1;
		}
		Location resourceLoc;
		if(resourceLevel!=getDataObject().levelID.get("User") && resourceLevel!=getDataObject().levelID.get("Mobile"))
			resourceLoc = getResourceLocationInfo(dataId);
		else
			resourceLoc = getUserLocationInfo(dataId,time);
		
		int parentInstanceId = References.NOT_SET;	
		String parentDataId = "";
				
	
		if(time<References.INIT_TIME){
			for(int i=0; i<getLevelWiseResources(parentLevel).size();i++){
				Location potentialParentLoc = getResourceLocationInfo(getLevelWiseResources(parentLevel).get(i));
				if(potentialParentLoc.block==resourceLoc.block) {
					parentDataId = getLevelWiseResources(parentLevel).get(i);
					for(int parentIdIterator: instanceToDataId.keySet())
					{
						if(instanceToDataId.get(parentIdIterator).equals(parentDataId))
						{
							parentInstanceId = parentIdIterator;
						}
					}
				}	
			}
		}
		else
		{
			if(parentLevel == 1) {
				getResourceLocationInfo(dataId).latitude = resourceLoc.latitude;
				getResourceLocationInfo(dataId).longitude = resourceLoc.longitude;
			}
			double minmumDistance = Config.MAX_VALUE;
			
			if(parentLevel == 2) {
				minmumDistance = Config.MAX_COMMUNICATION_RANGE / 1000;
			}
			
			int parentLevelMobile = -1;
			
			if(parentLevel == 2) {
				parentLevelMobile = 4;
			}
			for(int i=0; i<getLevelWiseResources(parentLevel).size();i++){
				Location potentialParentLoc = getResourceLocationInfo(getLevelWiseResources(parentLevel).get(i));
				
				double distance = calculateDistance(resourceLoc, potentialParentLoc);
					if(distance<minmumDistance){
						parentDataId = getLevelWiseResources(parentLevel).get(i);
						minmumDistance = distance;
					}
			}
			
			if(parentLevelMobile != -1) {
				for(int i=0; i<getLevelWiseResources(parentLevelMobile).size();i++){
					Location potentialParentLoc = getResourceLocationInfo(getLevelWiseResources(parentLevelMobile).get(i));
					
					double distance = calculateDistance(resourceLoc, potentialParentLoc);
						if(distance<minmumDistance){
							parentDataId = getLevelWiseResources(parentLevelMobile).get(i);
							minmumDistance = distance;
						}
				}
			}
			
			for(int parentIdIterator: instanceToDataId.keySet())
			{
				if(instanceToDataId.get(parentIdIterator).equals(parentDataId))
				{
					parentInstanceId = parentIdIterator;
				}
			}
			
			if(minmumDistance == Config.MAX_COMMUNICATION_RANGE) {
				System.out.println("não foi possivel encontrar um parente");
			}
			
		}
		
		if( parentInstanceId == -1) {
			System.out.println("não foi possivel encontrar um parente");
		}
		
		return parentInstanceId;	
	}	

	private Location getUserLocationInfo(String dataId, double time) {
		// TODO Auto-generated method stub
		return getDataObject().usersLocation.get(dataId).get(time);
	}
	
	//private Location getMobileResourceLocationInfo(String dataId, double time) {
	//	return getDataObject().mobileResourceLocation.get(dataId).get(time);
	//}

	private Location getResourceLocationInfo(String dataId) {
		// TODO Auto-generated method stub
		return getDataObject().resourceLocationData.get(dataId);
	}

	
	public List<Double> getTimeSheet(int instanceId) {
		
		String dataId = getDataIdByInstanceID(instanceId);
		List<Double>timeSheet = new ArrayList<Double>(getDataObject().usersLocation.get(dataId).keySet());
		return timeSheet;
	}

	public void linkDataWithInstance(int instanceId, String dataID) {
		// TODO Auto-generated method stub
		instanceToDataId.put(instanceId, dataID);
	}

	public int getLevelID(String resourceType) {
		// TODO Auto-generated method stub
		return dataObject.levelID.get(resourceType);
	}
	
	public ArrayList<String> getLevelWiseResources(int levelNo) {
		// TODO Auto-generated method stub
		return getDataObject().levelwiseResources.get(levelNo);
	}

	public void parseUserInfo(Map<Integer, Integer> userMobilityPattern, String datasetReference) throws IOException {
		// TODO Auto-generated method stub
		getDataObject().parseUserData(userMobilityPattern, datasetReference);
	}

	public void parseResourceInfo() throws NumberFormatException, IOException {
		// TODO Auto-generated method stub
		getDataObject().parseResourceData();
	}

	public List<String> getMobileUserDataId() {
		// TODO Auto-generated method stub
		List<String> userDataIds = new ArrayList<>(getDataObject().usersLocation.keySet());
		return userDataIds;
		
	}

	public Map<String, Integer> getDataIdsLevelReferences() {
		// TODO Auto-generated method stub
		return getDataObject().resourceAndUserToLevel;
	}
	
	public boolean isCloud(int instanceID) {
		// TODO Auto-generated method stub
		String dataId = getDataIdByInstanceID(instanceID);
		//System.out.println(dataId);
		int instenceLevel=getDataObject().resourceAndUserToLevel.get(dataId);
		if(instenceLevel==getDataObject().levelID.get("Cloud"))
			return true;
		else
			return false;
	}
	
	public String getDataIdByInstanceID(int instanceID) {
		// TODO Auto-generated method stub
		return instanceToDataId.get(instanceID);
	}
	
	public Map<Integer, String> getInstenceDataIdReferences() {
		// TODO Auto-generated method stub
		return instanceToDataId;
	}

	public boolean isAMobileDevice(int instanceId) {
		// TODO Auto-generated method stub
		String dataId = getDataIdByInstanceID(instanceId);
		int instenceLevel=getDataObject().resourceAndUserToLevel.get(dataId);
		if(instenceLevel== getDataObject().levelID.get("User") || instenceLevel== getDataObject().levelID.get("Mobile"))
			return true;
		else
			return false;
	}
}
