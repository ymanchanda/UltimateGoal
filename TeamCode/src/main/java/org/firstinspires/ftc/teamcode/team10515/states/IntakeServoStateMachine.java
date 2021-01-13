package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class IntakeServoStateMachine extends TimedState<IntakeServoStateMachine.State>{
    public IntakeServoStateMachine(){
        super(State.STANDBY);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Intake Servo State Machine";
    }

    public enum State implements Namable{
        STANDBY(0.0d), HIT_RING(0.5d);

        private final double position;

        State(final double position){
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        @Override
        public String getName(){
            return null;
        }
    }
}
