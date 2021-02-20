package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class ForkliftStateMachine2 extends TimedState<ForkliftStateMachine2.State> {
    public ForkliftStateMachine2() {
        super(State.DOWN);
    }

    @Override
    public String getName() {
        return "Forklift State Machine 2";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    public enum State implements Namable {
        DOWN(0), ALIGN_UP(45), ALIGN_DOWN(45), UP(155);

        private final double angle;

        State(final double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return this.angle;
        }

        @Override
        public String getName() {
            return null;
        }
    }

}
