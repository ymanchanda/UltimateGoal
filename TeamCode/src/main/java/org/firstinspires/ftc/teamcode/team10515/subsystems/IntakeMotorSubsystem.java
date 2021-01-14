package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;

public class IntakeMotorSubsystem implements ISubsystem<IntakeMotorStateMachine, IntakeMotorStateMachine.State> {
    private static IntakeMotorStateMachine intakeMotorStateMachine;
    private RevMotor intakeWheels;
    //private RevMotor leftFlywheel;
    //private RevMotor rightFlywheel;

    public IntakeMotorSubsystem(RevMotor intakeMotor){//RevMotor leftFlywheel, RevMotor rightFlywheel) {
        setIntakeMotorStateMachine(new IntakeMotorStateMachine());
       setIntakeWheels(intakeMotor);
//        setLeftFlywheel(leftFlywheel);
//        setRightFlywheel(rightFlywheel);
    }

    @Override
    public IntakeMotorStateMachine getStateMachine() {
        return intakeMotorStateMachine;
    }

    @Override
    public IntakeMotorStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
//        getLeftFlywheel().setPower(0d);
//        getRightFlywheel().setPower(0d);
        getIntakeWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getIntakeWheels().setPower(getState().getPower());
//        getLeftFlywheel().setPower(getState().getPower());
//        getRightFlywheel().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setIntakeMotorStateMachine(IntakeMotorStateMachine intakeMotorStateMachine) {
        IntakeMotorSubsystem.intakeMotorStateMachine = intakeMotorStateMachine;
    }

//    private RevMotor getLeftFlywheel() {
//        return leftFlywheel;
//    }
//
//    private void setLeftFlywheel(RevMotor leftFlywheel) {
//        this.leftFlywheel = leftFlywheel;
//    }
//
//    private RevMotor getRightFlywheel() {
//        return rightFlywheel;
//    }
//
//    private void setRightFlywheel(RevMotor rightFlywheel) {
//        this.rightFlywheel = rightFlywheel;
//    }

    private void setIntakeWheels(RevMotor intakeMotor){
        this.intakeWheels = intakeMotor;
    }
    private RevMotor getIntakeWheels(){
        return intakeWheels;
    }
}
