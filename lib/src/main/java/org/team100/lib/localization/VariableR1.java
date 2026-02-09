package org.team100.lib.localization;

/** Random variable in one dimension */
public class VariableR1 {
    private final double mean;
    private final double variance;

    public VariableR1(double mean, double variance) {
        this.mean = mean;
        this.variance = variance;
    }

    /**
     * Operands are independent, so means and variances simply add.
     */
    public static VariableR1 add(VariableR1 a, VariableR1 b) {
        return new VariableR1(a.mean + b.mean, a.variance + b.variance);
    }

    /**
     * Gaussian mixture model.
     * 
     * Uses inverse-variance weights but also includes a
     * covariance term for the dispersion of the means.
     * 
     * The variance is never less than the smaller component variance.
     */
    public static VariableR1 fuse1(VariableR1 a, VariableR1 b) {
        // double wA = weight(a.variance, b.variance);
        // double wB = weight(b.variance, a.variance);
        double wA = 1 / a.variance;
        double wB = 1 / b.variance;
        double totalWeight = wA + wB;
        double mean = (wA * a.mean + wB * b.mean) / totalWeight;
        // gaussian mixture
        // law of total variance
        // takes mean dispersion into account
        // https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians
        // https://stats.stackexchange.com/questions/309622/calculate-moments-of-a-weighted-mixture-of-normal-distributions
        double variance = 2 / totalWeight
                + wA * Math.pow(a.mean - mean, 2) / totalWeight
                + wB * Math.pow(b.mean - mean, 2) / totalWeight;
        // + wA * wB * Math.pow(a.mean - b.mean, 2) / Math.pow(totalWeight, 2);

        return new VariableR1(mean, variance);
    }

    /**
     * Uses inverse-variance weighting.
     */
    public static VariableR1 fuse2(VariableR1 a, VariableR1 b) {
        // TODO: handle zero
        double wA = 1 / a.variance;
        double wB = 1 / b.variance;
        double totalWeight = wA + wB;
        double mean = (wA * a.mean + wB * b.mean) / totalWeight;
        // inverse-variance weighting
        // does *not* take mean dispersion into account.
        // but does increase confidence with multiple measurements.
        // This is wrong, when the means are different: repeated updates
        // yield a slowly moving mean but rapidly increasing confidence
        // (which slows the movement of the mean)
        double variance = 1 / totalWeight;
        return new VariableR1(mean, variance);
    }

    /**
     * Covariance Inflation
     * 
     * Covariance is based on inverse variance weighting, with two terms for
     * covariance inflation:
     * 
     * * mean dispersion weight: reduce the influence of mean dispersion, but not to
     * zero.
     * * minimum state variance: avoid state variance collapse.
     * 
     * I tuned these terms by eye, in this sheet:
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=1604242948#gid=1604242948
     */
    public static VariableR1 fuse3(VariableR1 a, VariableR1 b) {
        // TODO: handle zero
        double wA = 1 / a.variance;
        double wB = 1 / b.variance;
        double totalWeight = wA + wB;
        double mean = (wA * a.mean + wB * b.mean) / totalWeight;

        // Inverse variance weight
        double variance = 1 / totalWeight;
        // Add (a little) mean dispersion, so that when very-different camera estimates
        // arrive, the state listens to them.
        double DISPERSION_WEIGHT = 0.02;
        variance += DISPERSION_WEIGHT * wA * Math.pow(a.mean - mean, 2) / totalWeight
                + DISPERSION_WEIGHT * wB * Math.pow(b.mean - mean, 2) / totalWeight;
        // Prevent variance collapse, so that the camera influence stays high
        // enough.
        double MIN_VARIANCE = 0.000009;
        variance = Math.max(variance, MIN_VARIANCE);
        return new VariableR1(mean, variance);
    }

    /**
     * Bayesian
     * 
     * https://stats.stackexchange.com/questions/237037/bayesian-updating-with-new-data
     * 
     * I think this post has an error in the mean computation, like they meant to
     * write precision (1/var).
     * 
     * https://seor.vse.gmu.edu/~klaskey/SYST664/Bayes_Unit5.pdf
     * 
     * Bayesian updating is identical to inverse variance weighting:
     * it weighs the more confident update more highly
     * it ignores mean dispersion
     */
    public static VariableR1 fuse4(VariableR1 a, VariableR1 b) {
        // double mean = (a.mean / a.variance + b.mean / b.variance) / (a.variance +
        // b.variance);
        double mean = (a.mean / a.variance + b.mean / b.variance) / (1 / a.variance + 1 / b.variance);
        // double variance = a.variance * b.variance / (a.variance + b.variance);
        double variance = 1 / (1 / a.variance + 1 / b.variance);
        return new VariableR1(mean, variance);
    }

    public static double weight(double varA, double varB) {
        if (varA < 1e-3 && varB < 1e-3) {
            return 0.5;
        }
        if (varA < 1e-3) {
            return 1.0;
        }
        if (varB < 1e-3) {
            return 0.0;
        }
        return (1 / varA) / (1 / varA + 1 / varB);
    }

    public double mean() {
        return mean;
    }

    public double variance() {
        return variance;
    }

}
