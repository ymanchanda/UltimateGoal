package org.firstinspires.ftc.teamcode.team10515.control;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ForkliftTorque {
    final double COUNTS_PER_MOTOR_REV = 2786.0;

    public double calculateAngle(double encoderTicks) {
        System.out.printf("Encoder ticks: %f\n", encoderTicks);

        encoderTicks /= COUNTS_PER_MOTOR_REV;
        encoderTicks *= 180;

        System.out.printf("Angle: %f\n", encoderTicks);

        return encoderTicks;
    }

    public double setAngle(double angle) {
        System.out.printf("Anlge: %f\n", angle);

        angle /= 180;
        angle *= COUNTS_PER_MOTOR_REV;

        System.out.printf("Encoder ticks: %f\n", angle);

        return angle;
    }

}
