package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class ForkliftStateMachine extends TimedState<ForkliftStateMachine.State> {
    public ForkliftStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Forklift State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    public enum State implements Namable {
        UP(0.7d), DOWN(-0.6d), IDLE(0.0d), HOLD(0d), AUTOUP(0.7d), AUTODOWN(-0.6d);

        private final double power;

        State(final double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }

        @Override
        public String getName() {
            return null;
        }
    }

}
