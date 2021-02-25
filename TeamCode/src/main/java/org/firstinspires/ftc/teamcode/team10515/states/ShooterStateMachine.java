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
        //use mm/sec
        HIGHGOAL("High Goal", 25000d),
        POLESHOT("Pole Shots", 22500d ),//23000, //24000d
        MIDGOAL("Middle Goal", 20000d),//21000
        LOWGOAL("Low Goal", 18000d),
        AUTO_EXTRA_SHOT("High Back", 24500);

        private final String name;
        private double speed;

        State(final String name, double speed) {
            this.name  = name;
            this.speed = speed;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getSpeed() {
            return this.speed;
        }

        public void setSpeed(double pSpeed) {
            this.speed = pSpeed;
        }
    }
}
