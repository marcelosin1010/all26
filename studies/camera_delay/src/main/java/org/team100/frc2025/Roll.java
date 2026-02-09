package org.team100.frc2025;

import java.util.function.ObjDoubleConsumer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Extract the roll component. */
public class Roll implements ObjDoubleConsumer<Transform3d> {
    private final ObjDoubleConsumer<Rotation2d> m_delegate;

    public Roll(ObjDoubleConsumer<Rotation2d> delegate) {
        m_delegate = delegate;
    }

    @Override
    public void accept(Transform3d t, double value) {
        m_delegate.accept(new Rotation2d(t.getRotation().getX()), value);
    }

}
