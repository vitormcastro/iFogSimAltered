/*
 * Title:        CloudSim Toolkit
 * Description:  CloudSim (Cloud Simulation) Toolkit for Modeling and Simulation of Clouds
 * Licence:      GPL - http://www.gnu.org/copyleft/gpl.html
 *
 * Copyright (c) 2009-2012, The University of Melbourne, Australia
 */

package main.java.org.cloudbus.cloudsim.util;

import java.util.List;

import main.java.org.cloudbus.cloudsim.Cloudlet;

/**
 * This interface defines what a workload model should provide. A workload model generates a list of
 * jobs that can be dispatched to a resource by {@link Workload}.
 * 
 * @author Marcos Dias de Assuncao
 * @since 5.0
 * 
 * @see Workload
 * @see WorkloadFileReader
 */
public interface WorkloadModel {

	/**
	 * Returns a list with the jobs generated by the workload.
	 * 
	 * @return a list with the jobs generated by the workload.
	 */
	List<Cloudlet> generateWorkload();

}
