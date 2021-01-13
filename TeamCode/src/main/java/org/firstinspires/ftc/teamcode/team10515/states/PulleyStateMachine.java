package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class PulleyStateMachine extends TimedState<PulleyStateMachine.State> {
    public PulleyStateMachine(){
        super(State.DOWN);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Pulley State Machine";
    }

    public enum State implements Namable{
        DOWN("Down", 0d, 0d), UP("Up", 0d, 0d);

        private final String name;
        private final double leftPosition;
        private final double rightPosition;

        State(final String name, final double leftPosition, final double rightPosition){
            this.name = name;
            this.leftPosition = leftPosition;
            this.rightPosition = rightPosition;
        }

        public double getLeftPosition() {
            return leftPosition;
        }

        public double getRightPosition() {
            return rightPosition;
        }

        @Override
        public String getName(){
            return name;
        }
    }
}