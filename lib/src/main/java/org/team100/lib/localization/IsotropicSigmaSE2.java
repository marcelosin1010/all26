package org.team100.lib.localization;

/**
 * Represents standard deviation in the situation we actually have, where x and
 * y are equivalent in every respect, and the covariances in SE(2) are assumed
 * to be zero for simplicity.  This leaves only two numbers.
 * 
 * @param cartesian Standard deviation of cartesian dimensions.
 * @param rotation  Standard deviation of rotation.
 */
public record IsotropicSigmaSE2(double cartesian, double rotation) {
}
