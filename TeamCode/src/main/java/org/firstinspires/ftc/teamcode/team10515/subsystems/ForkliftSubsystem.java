package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;

public class ForkliftSubsystem implements ISubsystem<ForkliftStateMachine, ForkliftStateMachine.State> {
    private static ForkliftStateMachine forkliftStateMachine;
    private RevMotor forkliftMotor;

    public ForkliftSubsystem(RevMotor forkliftMotor){
        setForkliftStateMachine(new ForkliftStateMachine());
        setForkliftMotor(forkliftMotor);
    }

    @Override
    public ForkliftStateMachine getStateMachine() {
        return forkliftStateMachine;
    }

    @Override
    public ForkliftStateMachine.State getState() {
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
        return "Forklift Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getForkliftMotor().setPower(getState().getPower());
//        getForkliftMotor().setTargetPosition(getForkliftMotor().getCurrentEncoderTicks() + 470);
//        getForkliftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (getForkliftMotor().getCurrentEncoderTicks()< 470) {
//            getForkliftMotor().setPower(getState().getPower());
//        }
    }

    public RevMotor getForkliftMotor(){
        return forkliftMotor;
    }

    public static void setForkliftStateMachine(ForkliftStateMachine forkliftStateMachine){
        ForkliftSubsystem.forkliftStateMachine = forkliftStateMachine;
    }

    public void setForkliftMotor(RevMotor motor){
        this.forkliftMotor = motor;
    }

}
