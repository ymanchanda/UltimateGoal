package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.control.ForkliftAngle;

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
        DOWN(0, 45), ALIGN_UP(45, 0), ALIGN_DOWN(45, 155), UP(155, 45);

        private final double angle, prevAngle;
        private final ForkliftAngle f = new ForkliftAngle(0.15, 0.6, 0.7);

        State(final double angle, final double prevAngle) {
            this.angle = angle;
            this.prevAngle = prevAngle;
            f.setAngle(angle, prevAngle);
        }

        public double getAngle(double ticks) {
            return f.getAngle(ticks);
        }

        public double getPrevAngle(){
            return prevAngle;
        }

        public double getPower(double ticks){
            return f.getSpeed(getAngle(ticks));
        }

        @Override
        public String getName() {
            return null;
        }
    }

}
