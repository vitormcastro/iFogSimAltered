package main.java.org.fog.mobilitydata;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import java.util.Random;
import main.java.org.fog.mobilitydata.References;

public class DataParser {
	public Map<String, Location> resourceLocationData = new HashMap<String, Location>();
	public Map<String, Integer> levelID = new HashMap<String, Integer>();
	public Map<Integer, ArrayList<String>> levelwiseResources = new HashMap<Integer, ArrayList<String>>();
	public Map<String, Integer> resourceAndUserToLevel = new HashMap<String, Integer>();
	public Map<String, Map<Double, Location>> usersLocation = new HashMap<String, Map<Double, Location>>();

	public DataParser() {
		File configFile = new File(".\\dataset\\config.properties");
		try {
			FileReader reader = new FileReader(configFile);
			Properties props = new Properties();
			props.load(reader);
			levelID.put("LevelsNum", Integer.parseInt(props.getProperty("Level")));
			levelID.put("Cloud", Integer.parseInt(props.getProperty("Cloud")));
			levelID.put("Proxy", Integer.parseInt(props.getProperty("Proxy")));
			levelID.put("Gateway", Integer.parseInt(props.getProperty("Gateway")));
			levelID.put("User", Integer.parseInt(props.getProperty("User")));
			levelID.put("Mobile", Integer.parseInt(props.getProperty("Mobile")));
			reader.close();
		} catch (FileNotFoundException ex) {
			// file does not exist
		} catch (IOException ex) {
			// I/O error
		}
	}

	private double nextMobilisationEvent(double eventTime, int mobilityPattern) {
		// TODO Auto-generated method stub
		Random ran = new Random();
		int seed;
		double newEventTime = -1;
		switch (mobilityPattern) {
		case References.DIRECTIONAL_MOBILITY:
			seed = 20;
			newEventTime = 1.00 + (double) ran.nextInt(seed) + eventTime;
			break;
		case References.RANDOM_MOBILITY:
			seed = 2000;
			newEventTime = 1.00 + (double) ran.nextInt(seed);
			break;
		}
		return newEventTime;

	}

	public void parseUserData(Map<Integer, Integer> userMobilityPattern, String datasetReference) throws IOException {
		// TODO Auto-generated method stub

		for (int userID : userMobilityPattern.keySet()) {

			Map<Double, Location> tempUserLocationInfo = new HashMap<Double, Location>();
			BufferedReader csvReader = new BufferedReader(new FileReader(datasetReference + userID + ".csv"));
			System.out.println("The Mobility dataset used in this simulation for user: " + userID + " is: "
					+ datasetReference + userID + ".csv");
			String row;
			double eventTime = References.INIT_TIME;
			while ((row = csvReader.readLine()) != null) {
				String[] data = row.split(",");
				try {
					Location rl = new Location(Double.parseDouble(data[0]), Double.parseDouble(data[1]),
							References.NOT_SET);
					if (!tempUserLocationInfo.containsKey(eventTime))
						tempUserLocationInfo.put(eventTime, rl);
					else {
						eventTime = nextMobilisationEvent(eventTime, userMobilityPattern.get(userID));
						tempUserLocationInfo.put(eventTime, rl);
					}

				} catch (NumberFormatException ex) {
					// System.out.println("Given String is not parsable to double");
				}
			}

			csvReader.close();
			usersLocation.put("usr_" + userID, tempUserLocationInfo);
			resourceAndUserToLevel.put("usr_" + userID, levelID.get("User"));

		}

	}

	public void parseMobileResourcerData(int resourceID, String datasetReference, ArrayList<String>[] resouresOnLevels, int block) throws IOException {
		// TODO Auto-generated method stub

		Map<Double, Location> tempUserLocationInfo = new HashMap<Double, Location>();
		BufferedReader csvReader = new BufferedReader(new FileReader(datasetReference + resourceID + ".csv"));
		System.out.println("The Mobility dataset used in this simulation for resource Mobile: " + resourceID + " is: "
				+ datasetReference + resourceID + ".csv");
		String row;
		double eventTime = References.INIT_TIME;
		while ((row = csvReader.readLine()) != null) {
			String[] data = row.split(",");
			try {
				Location rl = new Location(Double.parseDouble(data[0]), Double.parseDouble(data[1]), block);
				if (!tempUserLocationInfo.containsKey(eventTime)) {
					tempUserLocationInfo.put(eventTime, rl);
					tempUserLocationInfo.put(-1.0, rl);
				}
				else {
					eventTime = nextMobilisationEvent(eventTime, References.DIRECTIONAL_MOBILITY);
					tempUserLocationInfo.put(eventTime, rl);
				}
				
				
			} catch (NumberFormatException ex) {
				// System.out.println("Given String is not parsable to double");
			}
		}

		resourceLocationData.put("res_" + resourceID, tempUserLocationInfo.get(0.0));

		csvReader.close();
		usersLocation.put("res_" + resourceID, tempUserLocationInfo);
		resourceAndUserToLevel.put("res_" + resourceID, levelID.get("Mobile"));
		resouresOnLevels[4].add("res_" + resourceID);

	}

	@SuppressWarnings("unchecked")
	public void parseResourceData() throws NumberFormatException, IOException {

		int numOfLevels = levelID.get("LevelsNum") + 1;
		ArrayList<String>[] resouresOnLevels = new ArrayList[numOfLevels];
		for (int i = 0; i < numOfLevels; i++)
			resouresOnLevels[i] = new ArrayList<String>();

		BufferedReader csvReader = new BufferedReader(new FileReader(".\\dataset\\edgeResources-UFABCDatasetExpandido.csv"));//".\\dataset\\edgeResources-UFABCDatasetExpandido.csv"));
		String row;
		while ((row = csvReader.readLine()) != null) {
			String[] data = row.split(",");
			// System.out.println(row);
			if (data[6].equals("VIC")) {
				// System.out.println(row);
				Location rl = new Location(Double.parseDouble(data[1]), Double.parseDouble(data[2]),
						Integer.parseInt(data[3]));
				resouresOnLevels[Integer.parseInt(data[4])].add("res_" + data[0]);
				resourceAndUserToLevel.put("res_" + data[0], Integer.parseInt(data[4]));
				resourceLocationData.put("res_" + data[0], rl);
			} else if (data[6].equals("MOB")) {
				String reference = ".\\dataset\\mFogMoveLocation_";
				try {
				    int resourceId = Integer.parseInt(data[0]);
				    parseMobileResourcerData(resourceId, reference, resouresOnLevels, Integer.parseInt(data[3]));
				} catch (NumberFormatException e) {
				    System.out.println("Erro: a string não é um número válido.");
				}
				
			}
		}

		for (int i = 0; i < numOfLevels; i++) {
			levelwiseResources.put(i, resouresOnLevels[i]);
		}
		csvReader.close();
	}
}
