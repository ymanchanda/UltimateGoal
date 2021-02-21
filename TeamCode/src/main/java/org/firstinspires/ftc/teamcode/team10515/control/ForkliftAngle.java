package org.firstinspires.ftc.teamcode.team10515.control;

import org.firstinspires.ftc.teamcode.team10515.UltimateGoalRobot;

public class ForkliftAngle extends UltimateGoalRobot {

    final double COUNTS_PER_MOTOR_REV = 2786.0;

    private double minSpeed, maxSpeed, factorOfMS;
    private double initialAngle, currentAngle, targetAngle;
    private double angleDiff;

    public ForkliftAngle(double minSpeed, double maxSpeed, double percentageForMS){
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.factorOfMS = percentageForMS;
    }

    public double setAngle(double angle) {
        return setAngle(0, angle);
    }

    public double setAngle(double startingAngle, double angle) {
        initialAngle = startingAngle;

        System.out.printf("Target Angle: %f\n", angle);
        targetAngle = angle;

        angleDiff = targetAngle - initialAngle;

        System.out.printf("Encoder ticks: %f\n", angleDiff*COUNTS_PER_MOTOR_REV/180);

        return angleDiff*COUNTS_PER_MOTOR_REV/180;
    }

    public double calculateTicks(double angle) {
        return calculateTicks(0, angle);
    }

    public double calculateTicks(double startingAngle, double angle) {
        return (angle - startingAngle)*COUNTS_PER_MOTOR_REV/180;
    }

    public double getAngle(double encoderTicks) {
        System.out.printf("Encoder ticks: %f\n", encoderTicks);

        encoderTicks /= COUNTS_PER_MOTOR_REV;
        encoderTicks *= 180;

        System.out.printf("Angle: %f\n", encoderTicks);
        currentAngle = encoderTicks;

        return encoderTicks;//+initialAngle?
    }

    public double getSpeed(double ticks) {
        if(initialAngle == targetAngle) {
            return 0d;
        }
        currentAngle = getAngle(ticks);
        if(initialAngle < targetAngle) {
            telemetry.addLine("Going UP");
            if(currentAngle >= targetAngle) {
                return 0d;
            }
            if((currentAngle - initialAngle) <= factorOfMS * angleDiff){
                return maxSpeed;
            }
            else {
                return ((targetAngle - currentAngle)/((1-factorOfMS) * angleDiff)) * (maxSpeed - minSpeed) + minSpeed;
            }
        }
        else {
            telemetry.addLine("Going DOWN");
            if(currentAngle <= targetAngle) {
                return 0d;
            }
            if((currentAngle - initialAngle) >= factorOfMS * angleDiff){
                return -maxSpeed;
            }
            else {
                return (((currentAngle - targetAngle)/((1-factorOfMS) * angleDiff)) * (maxSpeed - minSpeed) - minSpeed);
            }
        }
    }

}

