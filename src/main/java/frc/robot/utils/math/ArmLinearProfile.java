package frc.robot.utils.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmLinearProfile {
    private final TrapezoidProfile xProfile;
    private final TrapezoidProfile yProfile;

    private ArmLinearProfile next;

    public ArmLinearProfile(TrapezoidProfile.Constraints constraints, Waypoint goal, Waypoint initial) {
        this(constraints, goal, initial, null);
    }

    public ArmLinearProfile(TrapezoidProfile.Constraints constraints, Waypoint goal, Waypoint initial, ArmLinearProfile next) {
        xProfile = new TrapezoidProfile(constraints,
                goal.xState,
                initial.xState);
        yProfile = new TrapezoidProfile(constraints,
                goal.yState,
                initial.yState);
        this.next = next;
    }

    public boolean hasFinished(double t) {
        double endTime = Math.max(xProfile.totalTime(), yProfile.totalTime());
        return t >= endTime;
    }

    public ArmLinearProfile getNext() {
        return next;
    }

    public ArmLinearProfile setNext(ArmLinearProfile next) {
        this.next = next;
        return this;
    }

    public Translation2d calculate(double t) {
        return new Translation2d(xProfile.calculate(t).position, yProfile.calculate(t).position);
    }

    public static class Waypoint {
        private final TrapezoidProfile.State xState;
        private final TrapezoidProfile.State yState;

        public Waypoint(TrapezoidProfile.State xState, TrapezoidProfile.State yState) {
            this.xState = xState;
            this.yState = yState;
        }

        public Waypoint(double x, double y, double vx, double vy) {
            this(new TrapezoidProfile.State(x, vx), new TrapezoidProfile.State(y, vy));
        }
    }
}
