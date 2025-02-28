package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine CenterCage2l4() {
        final AutoRoutine routine = m_factory.newRoutine("Center Cage 2L4");
        final AutoTrajectory path = routine.trajectory("centercage_2l4");

        routine.active().onTrue(
            path.resetOdometry().andThen(path.cmd())
        );

        return routine;
    }
}
