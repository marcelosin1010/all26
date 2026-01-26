package org.team100.lib.targeting;

/**
 * Shooting solution including time of flight.
 * 
 * @param elevation in radians
 * @param tof       in seconds
 */
public record FiringParameters(double elevation, double tof) {

}
