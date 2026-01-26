package org.team100.lib.targeting;

@FunctionalInterface
public interface IRange {
    /**
     * @param elevation in radians
     */
    FiringSolution get(double elevation);
}
