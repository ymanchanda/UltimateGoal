package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeServoStateMachine;

public class IntakeServoSubsystem implements ISubsystem<IntakeServoStateMachine, IntakeServoStateMachine.State>{
    private static IntakeServoStateMachine intakeServoStateMachine;
    private RevServo intakeServo;

    public IntakeServoSubsystem(RevServo servo){
        setIntakeServoStateMachine(new IntakeServoStateMachine());
        setServo(servo);
    }

    @Override
    public IntakeServoStateMachine getStateMachine() {
        return intakeServoStateMachine;
    }

    @Override
    public IntakeServoStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getServo().setPosition(0.0);

    }

    @Override
    public String getName() {
        return "Intake Servo Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getServo().setPosition(getState().getPosition());
    }

    public static void setIntakeServoStateMachine(IntakeServoStateMachine intakeServoStateMachine){
        IntakeServoSubsystem.intakeServoStateMachine = intakeServoStateMachine;
    }

    public RevServo getServo(){
        return intakeServo;
    }

    public void setServo(RevServo intakeServo){
        this.intakeServo = intakeServo;
    }
}
