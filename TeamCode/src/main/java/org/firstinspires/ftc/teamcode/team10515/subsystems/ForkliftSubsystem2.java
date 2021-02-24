package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.apache.commons.math3.random.RandomGenerator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;

public class ForkliftSubsystem2 implements ISubsystem<ForkliftStateMachine2, ForkliftStateMachine2.State> {
    private static ForkliftStateMachine2 ForkliftStateMachine2;
    private RevMotor forkliftMotor;
    final double COUNTS_PER_MOTOR_REV = 2786.0;
    private double minSpeed = 0.15, maxSpeed = 0.6, factorOfMS = 0.7;

    public ForkliftSubsystem2(RevMotor forkliftMotor){
        setForkliftStateMachine2(new ForkliftStateMachine2());
        setForkliftMotor(forkliftMotor);
    }

    @Override
    public ForkliftStateMachine2 getStateMachine() {
        return ForkliftStateMachine2;
    }

    @Override
    public ForkliftStateMachine2.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {    }

    @Override
    public void stop() {    }

    @Override
    public String getName() {
        return "Forklift Subsystem 2";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
    }

    public double getPower(){
        double power = 0d;
        double angleDiff = getAngle() - getState().getAngle();

        //return +- if within 5 degrees
        if(angleDiff <= 5 || angleDiff >= -5) {
            power = 0d;
        }
        else {
           if (angleDiff > 0) {
                if (angleDiff <= factorOfMS * angleDiff) {
                    power = maxSpeed;
                } else {
                    power = (angleDiff / ((1 - factorOfMS) * angleDiff)) * (maxSpeed - minSpeed) + minSpeed;
                }
            } else if (angleDiff < 0) {
                if (angleDiff >= factorOfMS * angleDiff) {
                    power = -maxSpeed;
                } else {
                    power = ((angleDiff / ((1 - factorOfMS) * angleDiff)) * (maxSpeed - minSpeed) - minSpeed);
                }
            }
        }
        return power;
    }

    public double getAngle() {
        double encoderTicks = forkliftMotor.getCurrentEncoderTicks();
        return encoderTicks/COUNTS_PER_MOTOR_REV*180;
    }
    public RevMotor getForkliftMotor(){
        return forkliftMotor;
    }

    public static void setForkliftStateMachine2(ForkliftStateMachine2 ForkliftStateMachine2){
        ForkliftSubsystem2.ForkliftStateMachine2 = ForkliftStateMachine2;
    }

    public void setForkliftMotor(RevMotor motor){
        this.forkliftMotor = motor;
    }

}
