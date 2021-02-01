package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class IntakeMotorStateMachine extends SimpleState<IntakeMotorStateMachine.State> {
    public IntakeMotorStateMachine() {

        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Intake Motor State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        //INTAKE("Intake", 0.9d),
        //OUTTAKE("Outtake", -0.7d);
        //use mm/sec
        INTAKE("Intake", 7000),
        OUTTAKE("Outtake", -5500);

        private final String name;
        private final double speed;

        State(final String name, final double speed) {
            this.name  = name;
            this.speed = speed;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getSpeed() {
            return speed;
        }
    }
}
