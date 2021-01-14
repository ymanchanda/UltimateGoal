package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;

public class PulleySubsystem implements ISubsystem<PulleyStateMachine, PulleyStateMachine.State> {
    public static PulleyStateMachine pulleyStateMachine;
    private RevServo leftServo;
    private RevServo rightServo;

    public PulleySubsystem(RevServo leftServo, RevServo rightServo){
        setPulleyStateMachine(new PulleyStateMachine());
    }

    @Override
    public PulleyStateMachine getStateMachine() {
        return pulleyStateMachine;
    }

    @Override
    public PulleyStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Pulley Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getLeftServo().setPosition(getState().getLeftPosition());
        getRightServo().setPosition(getState().getRightPosition());
    }

    public static void setPulleyStateMachine(PulleyStateMachine pulleyStateMachine){
        PulleySubsystem.pulleyStateMachine = pulleyStateMachine;
    }

    public RevServo getLeftServo(){
        return leftServo;
    }

    public RevServo getRightServo() {
        return rightServo;
    }

    public void setLeftServo(RevServo leftServo){
        this.leftServo = leftServo;
    }

    public void setRightServo(RevServo rightServo) {
        this.rightServo = rightServo;
    }
}
