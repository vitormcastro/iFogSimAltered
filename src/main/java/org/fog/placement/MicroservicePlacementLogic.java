package main.java.org.fog.placement;

import main.java.org.fog.application.Application;
import main.java.org.fog.entities.FogDevice;
import main.java.org.fog.entities.PlacementRequest;

import java.util.List;
import java.util.Map;

/**
 * Created by Samodha Pallewatta
 */
public interface MicroservicePlacementLogic {
    PlacementLogicOutput run(List<FogDevice> fogDevices, Map<String, Application> applicationInfo, Map<Integer, Map<String, Double>> resourceAvailability, List<PlacementRequest> pr);
    void updateResources(Map<Integer, Map<String, Double>> resourceAvailability);
    void postProcessing();
}
