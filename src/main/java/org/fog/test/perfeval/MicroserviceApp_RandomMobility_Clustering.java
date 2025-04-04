package main.java.org.fog.test.perfeval;

import org.apache.commons.math3.util.Pair;
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
import main.java.org.fog.entities.*;
import main.java.org.fog.entities.MicroserviceFogDevice;
import main.java.org.fog.entities.PlacementRequest;
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
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.*;

/**
 * Simulation setup for Microservices Application
 * This test covers featured such as,
 * 1. creation of clusters among fog nodes using dynamic clustering
 * 2. mobility of end user devices and microservice migration
 *
 * @author Samodha Pallewatta
 */

/**
 * Config properties
 * SIMULATION_MODE -> static
 * PR_PROCESSING_MODE -> PERIODIC
 * ENABLE_RESOURCE_DATA_SHARING -> false (not needed as FONs placed at the highest level.
 */
public class MicroserviceApp_RandomMobility_Clustering {
    static List<FogDevice> fogDevices = new ArrayList<FogDevice>();
    static List<Sensor> sensors = new ArrayList<Sensor>();
    static List<Actuator> actuators = new ArrayList<Actuator>();

    static Map<Integer, Integer> userMobilityPattern = new HashMap<Integer, Integer>();
    static LocationHandler locator;

    static double SENSOR_TRANSMISSION_TIME = 10;
    static int numberOfMobileUser = 1;

    // if random mobility generator for users is True, new random dataset will be created for each user
    static boolean randomMobility_generator = false; // To use random datasets
    static boolean renewDataset = false; // To overwrite existing random datasets
    static List<Integer> clusteringLevels = new ArrayList<Integer>(); // The selected fog layers for clustering

    //application
    static List<Application> applications = new ArrayList<>();
    static List<Pair<Double, Double>> qosValues = new ArrayList<>();

    public static void main(String[] args) {

        try {

            Log.disable();
            int num_user = 1; // number of cloud users
            Calendar calendar = Calendar.getInstance();
            boolean trace_flag = false; // mean trace events

            CloudSim.init(num_user, calendar, trace_flag);

            FogBroker broker = new FogBroker("broker");

            /**
             * Microservices-based application creation - a single application is selected for this
             */
            Application microservicesApplication = createApplication("example", broker.getId());
            applications.add(microservicesApplication);

            //
            DataParser dataObject = new DataParser();
            locator = new LocationHandler(dataObject);

            String datasetReference = References.dataset_reference;

            if (randomMobility_generator) {
                datasetReference = References.dataset_random;
                createRandomMobilityDatasets(References.random_walk_mobility_model, datasetReference, renewDataset);
            }

            /**
             * Clustered Fog node creation.
             * 01. Create devices (Client,FON,FCN,Cloud)
             * 02. Generate cluster connection.
             * 03. Identify devices monitored by each FON
             */
            createMobileUser(broker.getId(), applications.get(0).getAppId(), datasetReference);
            createFogDevices(broker.getId(), applications.get(0).getAppId());

            /**
             * Central controller for performing preprocessing functions
             */
            List<Application> appList = new ArrayList<>();
            for (Application application : applications)
                appList.add(application);


            List<Integer> clusterLevelIdentifier = new ArrayList<>();
            clusterLevelIdentifier.add(2);

            int placementAlgo = PlacementLogicFactory.CLUSTERED_MICROSERVICES_PLACEMENT;
            MicroservicesMobilityClusteringController microservicesController = new MicroservicesMobilityClusteringController("controller", fogDevices, sensors, appList, clusterLevelIdentifier, 2.0, placementAlgo, locator);


            // generate placement requests
            List<PlacementRequest> placementRequests = new ArrayList<>();
            for (Sensor s : sensors) {
                Map<String, Integer> placedMicroservicesMap = new HashMap<>();
                placedMicroservicesMap.put("clientModule", s.getGatewayDeviceId());
                PlacementRequest p = new PlacementRequest(s.getAppId(), s.getId(), s.getGatewayDeviceId(), placedMicroservicesMap);
                placementRequests.add(p);
            }

            microservicesController.submitPlacementRequests(placementRequests, 0);


            TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());

            CloudSim.startSimulation();

            CloudSim.stopSimulation();

            Log.printLine("VRGame finished!");
        } catch (Exception e) {
            e.printStackTrace();
            Log.printLine("Unwanted errors happen");
        }
    }


    private static void createRandomMobilityDatasets(int mobilityModel, String datasetReference, boolean renewDataset) throws IOException, ParseException {
        RandomMobilityGenerator randMobilityGenerator = new RandomMobilityGenerator();
        for (int i = 0; i < numberOfMobileUser; i++) {

            randMobilityGenerator.createRandomData(mobilityModel, i + 1, datasetReference, renewDataset);
        }
    }

    /**
     * Creates the fog devices in the physical topology of the simulation.
     *
     * @param userId
     */
    private static void createFogDevices(int userId, String appId) throws NumberFormatException, IOException {
        locator.parseResourceInfo();


        if (locator.getLevelWiseResources(locator.getLevelID("Cloud")).size() == 1) {

            FogDevice cloud = createFogDevice("cloud", 44800, 40000, 100, 10000, 0.01, 16 * 103, 16 * 83.25, MicroserviceFogDevice.CLOUD); // creates the fog device Cloud at the apex of the hierarchy with level=0
            cloud.setParentId(References.NOT_SET);
            locator.linkDataWithInstance(cloud.getId(), locator.getLevelWiseResources(locator.getLevelID("Cloud")).get(0));
            cloud.setLevel(0);
            fogDevices.add(cloud);

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Proxy")).size(); i++) {

                FogDevice proxy = createFogDevice("proxy-server_" + i, 2800, 4000, 10000, 10000, 0.0, 107.339, 83.4333, MicroserviceFogDevice.FON); // creates the fog device Proxy Server (level=1)
                locator.linkDataWithInstance(proxy.getId(), locator.getLevelWiseResources(locator.getLevelID("Proxy")).get(i));
                proxy.setParentId(cloud.getId()); // setting Cloud as parent of the Proxy Server
                proxy.setUplinkLatency(100); // latency of connection from Proxy Server to the Cloud is 100 ms
                proxy.setLevel(1);
                fogDevices.add(proxy);

            }

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Gateway")).size(); i++) {

                FogDevice gateway = createFogDevice("gateway_" + i, 2800, 4000, 10000, 10000, 0.0, 107.339, 83.4333, MicroserviceFogDevice.FCN);
                locator.linkDataWithInstance(gateway.getId(), locator.getLevelWiseResources(locator.getLevelID("Gateway")).get(i));
                gateway.setParentId(locator.determineParent(gateway.getId(), References.SETUP_TIME));
                gateway.setUplinkLatency(4);
                gateway.setLevel(2);
                fogDevices.add(gateway);
            }

        }
    }

    private static void createMobileUser(int userId, String appId, String datasetReference) throws IOException {

        for (int id = 1; id <= numberOfMobileUser; id++)
            userMobilityPattern.put(id, References.DIRECTIONAL_MOBILITY);

        locator.parseUserInfo(userMobilityPattern, datasetReference);

        List<String> mobileUserDataIds = locator.getMobileUserDataId();

        for (int i = 0; i < numberOfMobileUser; i++) {
            FogDevice mobile = addMobile("mobile_" + i, userId, appId, References.NOT_SET); // adding mobiles to the physical topology. Smartphones have been modeled as fog devices as well.
            mobile.setUplinkLatency(2); // latency of connection between the smartphone and proxy server is 2 ms
            locator.linkDataWithInstance(mobile.getId(), mobileUserDataIds.get(i));
            mobile.setLevel(3);

            fogDevices.add(mobile);
        }

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
                    new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 1250000, 0, ratePerMips, deviceType);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return fogdevice;
    }

    private static FogDevice addMobile(String name, int userId, String appId, int parentId) {
        FogDevice mobile = createFogDevice(name, 500, 20, 1000, 270, 0, 87.53, 82.44, MicroserviceFogDevice.CLIENT);
        mobile.setParentId(parentId);
        //locator.setInitialLocation(name,drone.getId());
        Sensor mobileSensor = new Sensor("sensor-" + name, "M-SENSOR", userId, appId, new DeterministicDistribution(SENSOR_TRANSMISSION_TIME)); // inter-transmission time of EEG sensor follows a deterministic distribution
        mobileSensor.setApp(applications.get(0));
        sensors.add(mobileSensor);
        Actuator mobileDisplay = new Actuator("actuator-" + name, userId, appId, "M-DISPLAY");
        actuators.add(mobileDisplay);

        mobileSensor.setGatewayDeviceId(mobile.getId());
        mobileSensor.setLatency(6.0);  // latency of connection between EEG sensors and the parent Smartphone is 6 ms

        mobileDisplay.setGatewayDeviceId(mobile.getId());
        mobileDisplay.setLatency(1.0);  // latency of connection between Display actuator and the parent Smartphone is 1 ms
        mobileDisplay.setApp(applications.get(0));

        return mobile;
    }

    private static Application createApplication(String appId, int userId) {
        Application application = Application.createApplication(appId, userId); // creates an empty application model (empty directed graph)

        /*
         * Adding modules (vertices) to the application model (directed graph)
         */
        application.addAppModule("clientModule", 10); // adding module Client to the application model
        application.addAppModule("processingModule", 10); // adding module Concentration Calculator to the application model
        application.addAppModule("storageModule", 10); // adding module Connector to the application model

        /*
         * Connecting the application modules (vertices) in the application model (directed graph) with edges
         */
        if (SENSOR_TRANSMISSION_TIME == 5.1)
            application.addAppEdge("M-SENSOR", "clientModule", 2000, 500, "M-SENSOR", Tuple.UP, AppEdge.SENSOR); // adding edge from EEG (sensor) to Client module carrying tuples of type EEG
        else
            application.addAppEdge("M-SENSOR", "clientModule", 3000, 500, "M-SENSOR", Tuple.UP, AppEdge.SENSOR);
        application.addAppEdge("clientModule", "processingModule", 3500, 500, "RAW_DATA", Tuple.UP, AppEdge.MODULE); // adding edge from Client to Concentration Calculator module carrying tuples of type _SENSOR
        application.addAppEdge("processingModule", "storageModule", 1000, 1000, "PROCESSED_DATA", Tuple.UP, AppEdge.MODULE); // adding periodic edge (period=1000ms) from Concentration Calculator to Connector module carrying tuples of type PLAYER_GAME_STATE
        application.addAppEdge("processingModule", "clientModule", 14, 500, "ACTION_COMMAND", Tuple.DOWN, AppEdge.MODULE);  // adding edge from Concentration Calculator to Client module carrying tuples of type CONCENTRATION
        application.addAppEdge("clientModule", "M-DISPLAY", 1000, 500, "ACTUATION_SIGNAL", Tuple.DOWN, AppEdge.ACTUATOR);  // adding edge from Client module to Display (actuator) carrying tuples of type SELF_STATE_UPDATE

        /*
         * Defining the input-output relationships (represented by selectivity) of the application modules.
         */
        application.addTupleMapping("clientModule", "M-SENSOR", "RAW_DATA", new FractionalSelectivity(1.0)); // 0.9 tuples of type _SENSOR are emitted by Client module per incoming tuple of type EEG
        application.addTupleMapping("processingModule", "RAW_DATA", "PROCESSED_DATA", new FractionalSelectivity(1.0)); // 1.0 tuples of type SELF_STATE_UPDATE are emitted by Client module per incoming tuple of type CONCENTRATION
        application.addTupleMapping("processingModule", "RAW_DATA", "ACTION_COMMAND", new FractionalSelectivity(1.0)); // 1.0 tuples of type CONCENTRATION are emitted by Concentration Calculator module per incoming tuple of type _SENSOR
        application.addTupleMapping("clientModule", "ACTION_COMMAND", "ACTUATION_SIGNAL", new FractionalSelectivity(1.0)); // 1.0 tuples of type GLOBAL_STATE_UPDATE are emitted by Client module per incoming tuple of type GLOBAL_GAME_STATE

        application.setSpecialPlacementInfo("storageModule", "cloud");
        /*
         * Defining application loops to monitor the latency of.
         * Here, we add only one loop for monitoring : EEG(sensor) -> Client -> Concentration Calculator -> Client -> DISPLAY (actuator)
         */
        final AppLoop loop1 = new AppLoop(new ArrayList<String>() {{
            add("M-SENSOR");
            add("clientModule");
            add("processingModule");
            add("clientModule");
            add("M-DISPLAY");
        }});
        List<AppLoop> loops = new ArrayList<AppLoop>() {{
            add(loop1);
        }};
        application.setLoops(loops);

        return application;
    }


}