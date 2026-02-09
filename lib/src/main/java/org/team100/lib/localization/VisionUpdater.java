package org.team100.lib.localization;

import edu.wpi.first.math.geometry.Pose2d;

/** For testing. */
public interface VisionUpdater {

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    void put(
            double timestampS,
            Pose2d measurement,
            IsotropicNoiseSE2 visionSigma);

}