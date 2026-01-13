package org.team100.lib.util;

/**
 * Math to support indirect measurment of the position a large gear (e.g. a
 * turret).
 * 
 * The general idea is to use two encoders, each geared to the large gear using
 * different, coprime ratios.
 * 
 * So, given sensor gear tooth counds m1 and m2, we want to find x (the big gear
 * position), where we only know the remainders, a1 and a2:
 * 
 * x mod m1 = a1
 * x mod m2 = a2
 * 
 * This is similar to the "Chinese Remainder Theorem," but in this case the
 * measurements are continuous, not integers.
 */
@SuppressWarnings("unused")
public class IndirectDualMeasurement {

    /** final gear teeth */
    private final int m_N;
    /** sensor 1 gear teeth */
    private final int m_m1;
    /** sensor 2 gear teeth */
    private final int m_m2;

    /**
     * @param N  final gear teeth
     * @param m1 sensor 1 gear teeth
     * @param m2 sensor 2 gear teeth
     */
    public IndirectDualMeasurement(int N, int m1, int m2) {
        m_N = N;
        m_m1 = m1;
        m_m2 = m2;
    }

    /**
     * @param a1 sensor 1 measurement in teeth
     * @param a2 sensor 2 measurement in teeth
     */
    public double x(double a1, double a2) {
        return 0;
    }

}
