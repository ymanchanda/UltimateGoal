package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class ShooterStateMachine extends SimpleState<ShooterStateMachine.State>{

    public ShooterStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Shooter State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        //SPEED1("High Goal", 0.6d),
        //SPEED2("Pole Shots", 0.53d);
        //use mm/sec
        SPEED1("High Goal", 25000d),
        SPEED2("Pole Shots", 22000d),
        SPEED3("Middle Goal", 21000d);

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
