package main.java.org.fog.simulation;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.json.simple.parser.ParseException;

import main.java.org.cloudbus.cloudsim.Host;
import main.java.org.cloudbus.cloudsim.Log;
import main.java.org.cloudbus.cloudsim.Pe;
import main.java.org.cloudbus.cloudsim.Storage;
import main.java.org.cloudbus.cloudsim.core.CloudSim;
import main.java.org.cloudbus.cloudsim.power.PowerHost;
import main.java.org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import main.java.org.cloudbus.cloudsim.sdn.overbooking.BwProvisionerOverbooking;
import main.java.org.cloudbus.cloudsim.sdn.overbooking.PeProvisionerOverbooking;
import main.java.org.fog.application.AppEdge;
import main.java.org.fog.application.AppLoop;
import main.java.org.fog.application.Application;
import main.java.org.fog.application.selectivity.FractionalSelectivity;
import main.java.org.fog.entities.Actuator;
import main.java.org.fog.entities.FogBroker;
import main.java.org.fog.entities.FogDevice;
import main.java.org.fog.entities.FogDeviceCharacteristics;
import main.java.org.fog.entities.MicroserviceFogDevice;
import main.java.org.fog.entities.PlacementRequest;
import main.java.org.fog.entities.Sensor;
import main.java.org.fog.entities.Tuple;
import main.java.org.fog.mobilitydata.DataParser;
import main.java.org.fog.mobilitydata.RandomMobilityGenerator;
import main.java.org.fog.mobilitydata.References;
import main.java.org.fog.placement.LocationHandler;
import main.java.org.fog.placement.MicroservicesMobilityClusteringController;
import main.java.org.fog.placement.PlacementLogicFactory;
import main.java.org.fog.policy.AppModuleAllocationPolicy;
import main.java.org.fog.scheduler.StreamOperatorScheduler;
import main.java.org.fog.utils.FogLinearPowerModel;
import main.java.org.fog.utils.FogUtils;
import main.java.org.fog.utils.TimeKeeper;
import main.java.org.fog.utils.distribution.DeterministicDistribution;

public class SmartBikeClusteringAndMicro {
	static List<FogDevice> fogDevices = new ArrayList<FogDevice>();
    static List<Sensor> sensors = new ArrayList<Sensor>();
    
    static Map<Integer, Integer> userMobilityPattern = new HashMap<Integer, Integer>();
    static LocationHandler locator;
    
    static boolean CLOUD = false;

    static double SENSOR_TRANSMISSION_TIME = 10;
    static int numberOfMobileUser =30;
    
  //cluster link latency 2ms
    static Double clusterLatency = 2.0;
    
 // if random mobility generator for users is True, new random dataset will be created for each user
    static boolean randomMobility_generator = true; // To use random datasets
    static boolean renewDataset = true; // To overwrite existing random datasets
    static List<Integer> clusteringLevels = new ArrayList<Integer>(); // The selected fog layers for clustering

    public static void main(String[] args) {

    	 Log.printLine("Starting Smart Bike Application...");
    	 
    	 try {
    		 
    		 Log.disable();
             int num_user = 1; // number of cloud users
             Calendar calendar = Calendar.getInstance();
             boolean trace_flag = false; // mean trace events
             
             CloudSim.init(num_user, calendar, trace_flag);
             
             String appId = "Smart Bike Application SB";
             
             FogBroker broker = new FogBroker("broker");

             Application application = createApplication(appId, broker.getId());
             application.setUserId(broker.getId());
             
             DataParser dataObject = new DataParser();
             locator = new LocationHandler(dataObject);

             String datasetReference = ".\\dataset\\usersLocation-UFABC_";

             if (randomMobility_generator) {
                 datasetReference = References.dataset_random;
                 createRandomMobilityDatasets(References.random_walk_mobility_model, datasetReference, renewDataset);
             }
             
             createMobileUser(broker.getId(), application, datasetReference);
             createFogDevices(broker.getId(), application);
             
             List<Integer> clusterLevelIdentifier = new ArrayList<>();
             clusterLevelIdentifier.add(2);
             
             /**
              * Central controller for performing preprocessing functions
              */
             List<Application> appList = new ArrayList<>();
             appList.add(application);

             int placementAlgo = PlacementLogicFactory.CLUSTERED_MICROSERVICES_PLACEMENT;
             MicroservicesMobilityClusteringController microservicesController = new MicroservicesMobilityClusteringController("controller", fogDevices, sensors, appList, clusterLevelIdentifier, clusterLatency, placementAlgo,locator);

             List<PlacementRequest> placementRequests = new ArrayList<>();
             for (Sensor s : sensors) {
                 Map<String, Integer> placedMicroservicesMap = new HashMap<>();
                 placedMicroservicesMap.put("clientModule", s.getGatewayDeviceId());
                 PlacementRequest p = new PlacementRequest(s.getAppId(), s.getId(), s.getGatewayDeviceId(), placedMicroservicesMap);
                 placementRequests.add(p);
             }
             
             microservicesController.submitPlacementRequests(placementRequests, 1);

             TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());

             CloudSim.startSimulation();

             CloudSim.stopSimulation();

             Log.printLine("SB app finished!");
             
    	 }catch(Exception e) {
             e.printStackTrace();
             Log.printLine("Unwanted errors happen");
         }
    	
    }
    
    @SuppressWarnings({"serial"})
    private static Application createApplication(String appId, int userId) {

        Application application = Application.createApplication(appId, userId);

        /*
         * Adding modules (vertices) to the application model (directed graph)
         */
        application.addAppModule("clientModule", 1, 19, 1);
        application.addAppModule("mService1", 512, 250, 200);
        application.addAppModule("mService2", 512,150,2048);
       

        /*
         * Connecting the application modules (vertices) in the application model (directed graph) with edges
         */

        application.addAppEdge("SENSOR", "clientModule", 19, 1, "SENSOR", Tuple.UP, AppEdge.SENSOR);
        application.addAppEdge("clientModule", "mService1", 2000, 500, "RAW_DATA", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("mService1", "mService2", 1000, 500, "PROCESSED_DATA", Tuple.UP, AppEdge.MODULE);
       

        /*
         * Defining the input-output relationships (represented by selectivity) of the application modules.
         */
        application.addTupleMapping("clientModule", "SENSOR", "RAW_DATA", new FractionalSelectivity(0.9));
        application.addTupleMapping("mService1", "RAW_DATA", "PROCESSED_DATA", new FractionalSelectivity(1.0));
       
        application.setSpecialPlacementInfo("mService2", "cloud");
        if (CLOUD) {
            application.setSpecialPlacementInfo("mService1", "cloud");
        }

        final AppLoop loop1 = new AppLoop(new ArrayList<String>() {{
            add("SENSOR");
            add("clientModule");
            add("mService1");
            add("mService2");
        }});

        List<AppLoop> loops = new ArrayList<AppLoop>() {{
            add(loop1);
        }};
        application.setLoops(loops);


        application.createDAG();

        return application;
    }
    
    private static void createRandomMobilityDatasets(int mobilityModel, String datasetReference, boolean renewDataset) throws IOException, ParseException {
        RandomMobilityGenerator randMobilityGenerator = new RandomMobilityGenerator();
        for (int i = 0; i < numberOfMobileUser; i++) {

            randMobilityGenerator.createRandomData(mobilityModel, i + 1, datasetReference, renewDataset);
        }
    }
    
    private static void createMobileUser(int userId, Application app, String datasetReference) throws IOException {

        for (int id = 1; id <= numberOfMobileUser; id++)
            userMobilityPattern.put(id, References.DIRECTIONAL_MOBILITY);

        locator.parseUserInfo(userMobilityPattern, datasetReference);

        List<String> mobileUserDataIds = locator.getMobileUserDataId();

        for (int i = 0; i < numberOfMobileUser; i++) {
            FogDevice mobile = addMobile("mobile_" + i, userId, app, References.NOT_SET); // adding mobiles to the physical topology. Smartphones have been modeled as fog devices as well.
            mobile.setUplinkLatency(2); // latency of connection between the smartphone and proxy server is 2 ms
            locator.linkDataWithInstance(mobile.getId(), mobileUserDataIds.get(i));
            mobile.setLevel(3);

            fogDevices.add(mobile);
        }

    }
    
    /**
     * Creates the fog devices in the physical topology of the simulation.
     *
     * @param userId
     */
    private static void createFogDevices(int userId, Application app) throws NumberFormatException, IOException {
        locator.parseResourceInfo();


        if (locator.getLevelWiseResources(locator.getLevelID("Cloud")).size() == 1) {

            FogDevice cloud = createFogDevice("cloud", 44800, 256000, 100000, 100000, 0.01, 16 * 107.339, 16 * 83.433, 
            		MicroserviceFogDevice.CLOUD); // creates the fog device Cloud at the apex of the hierarchy with level=0
            cloud.setParentId(References.NOT_SET);
            locator.linkDataWithInstance(cloud.getId(), locator.getLevelWiseResources(locator.getLevelID("Cloud")).get(0));
            cloud.setLevel(0);
            fogDevices.add(cloud);

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Proxy")).size(); i++) {

                FogDevice proxy = createFogDevice("proxy-server_" + i, 3000, 16000, 100000, 100000, 
                		0.0, 107.339, 83.4333, MicroserviceFogDevice.FON); // creates the fog device Proxy Server (level=1)
                locator.linkDataWithInstance(proxy.getId(), locator.getLevelWiseResources(locator.getLevelID("Proxy")).get(i));
                proxy.setParentId(cloud.getId()); // setting Cloud as parent of the Proxy Server
                proxy.setUplinkLatency(100); // latency of connection from Proxy Server to the Cloud is 100 ms
                proxy.setLevel(1);
                fogDevices.add(proxy);

            }

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Gateway")).size(); i++) {

                FogDevice gateway = createFogDevice("gateway_" + i, 3000, 8000, 50000, 100000, 
                		0.0, 107.339, 83.4333, MicroserviceFogDevice.FCN);
                locator.linkDataWithInstance(gateway.getId(), locator.getLevelWiseResources(locator.getLevelID("Gateway")).get(i));
                gateway.setParentId(locator.determineParent(gateway.getId(), References.SETUP_TIME));
                gateway.setUplinkLatency(4);
                gateway.setLevel(2);
                fogDevices.add(gateway);
            }
            
            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Mobile")).size(); i++) {
            	FogDevice gateway = createFogDevice("mFog_" + i, 2800, 4000, 10000, 10000, 0.0, 107.339, 83.4333, MicroserviceFogDevice.FCN);
                locator.linkDataWithInstance(gateway.getId(), locator.getLevelWiseResources(locator.getLevelID("Mobile")).get(i));
                gateway.setParentId(locator.determineParent(gateway.getId(), References.SETUP_TIME));
                gateway.setUplinkLatency(4);
                gateway.setLevel(2);
                fogDevices.add(gateway);
            }

        }
    }
    
    private static FogDevice addMobile(String name, int userId, Application app, int parentId) {
        FogDevice mobile = createFogDevice(name, 64, 4, 24000, 48000, 0, 0.000000612, 0.000000045, 
        		MicroserviceFogDevice.CLIENT);
        mobile.setParentId(parentId);
        //locator.setInitialLocation(name,drone.getId());
        Sensor mobileSensor = new Sensor("s-" + name, "SENSOR", userId, app.getAppId(),
        		new DeterministicDistribution(SENSOR_TRANSMISSION_TIME)); // inter-transmission time of EEG sensor follows a deterministic distribution
        mobileSensor.setApp(app);
        sensors.add(mobileSensor);
        
        mobileSensor.setGatewayDeviceId(mobile.getId());
        mobileSensor.setLatency(6.0);  // latency of connection between EEG sensors and the parent Smartphone is 6 ms       

        return mobile;
    }
    
    /**
     * Creates a vanilla fog device
     *
     * @param nodeName    name of the device to be used in simulation
     * @param mips        MIPS
     * @param ram         RAM
     * @param upBw        uplink bandwidth
     * @param downBw      downlink bandwidth
     * @param ratePerMips cost rate per MIPS used
     * @param busyPower
     * @param idlePower
     * @return
     */
    private static MicroserviceFogDevice createFogDevice(String nodeName, long mips,
                                                         int ram, long upBw, long downBw, double ratePerMips, double busyPower, double idlePower, String deviceType) {

        List<Pe> peList = new ArrayList<Pe>();

        // 3. Create PEs and add these into a list.
        peList.add(new Pe(0, new PeProvisionerOverbooking(mips))); // need to store Pe id and MIPS Rating

        int hostId = FogUtils.generateEntityId();
        long storage = 1000000; // host storage
        int bw = 10000;

        PowerHost host = new PowerHost(
                hostId,
                new RamProvisionerSimple(ram),
                new BwProvisionerOverbooking(bw),
                storage,
                peList,
                new StreamOperatorScheduler(peList),
                new FogLinearPowerModel(busyPower, idlePower)
        );

        List<Host> hostList = new ArrayList<Host>();
        hostList.add(host);

        String arch = "x86"; // system architecture
        String os = "Linux"; // operating system
        String vmm = "Xen";
        double time_zone = 10.0; // time zone this resource located
        double cost = 3.0; // the cost of using processing in this resource
        double costPerMem = 0.05; // the cost of using memory in this resource
        double costPerStorage = 0.001; // the cost of using storage in this
        // resource
        double costPerBw = 0.0; // the cost of using bw in this resource
        LinkedList<Storage> storageList = new LinkedList<Storage>(); // we are not adding SAN
        // devices by now

        FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics(
                arch, os, vmm, host, time_zone, cost, costPerMem,
                costPerStorage, costPerBw);

        MicroserviceFogDevice fogdevice = null;
        try {
            fogdevice = new MicroserviceFogDevice(nodeName, characteristics,
                    new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 10000, 0, ratePerMips, deviceType);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return fogdevice;
    }

    
}

