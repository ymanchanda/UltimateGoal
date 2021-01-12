package org.firstinspires.ftc.teamcode.team10515;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;
//
//import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
//import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

@TeleOp(name = "Intake Test", group = "Test")

public class IntakeTest extends OpMode {
    UGMapTest robot = new UGMapTest();
    public double intakeMotorSpeed = 0;
    public double incrementspeed = 0.5;
    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;
    public ElapsedTime btnPressedB;

    public int inTakeStep = 0;
    private ElapsedTime runtime;
    private double expiredTime = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Robot OpMode starting: ", "Intake Test");
        updateTelemetry(telemetry);
        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        if (getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
            telemetry.addLine("a pressed");
            intakeMotorSpeed += incrementspeed;
            btnPressedA.reset();
            intakeMotorSpeed = Range.clip(intakeMotorSpeed, 6, 8.5);
        } else if (getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250) {
            telemetry.addLine("y pressed");
            intakeMotorSpeed -= incrementspeed;
            btnPressedY.reset();
            lastTimeY = currentTimeMillis();
            intakeMotorSpeed = Range.clip(intakeMotorSpeed, 6, 8.5);
        } else if (getEnhancedGamepad1().isRight_bumper())
        {
            intakeMotorSpeed = 0;
        }

        robot.intakeMotor.setPower(intakeMotorSpeed);
        telemetry.addLine("Intake Motor Speed: " + intakeMotorSpeed);

        if (getEnhancedGamepad1().isB() && btnPressedB.milliseconds() > 250) {
            inTakeStep = 0;
            runIntake();
        }
        runIntake();
    }
    private void runIntake() {
        switch (inTakeStep) {
            case 0:
                robot.intakeServo.setPosition(0.6);
                expiredTime = runtime.time() + 200;
                inTakeStep=1;
                break;
            case 1:
                if (runtime.time() > expiredTime) {
                    robot.intakeServo.setPosition(0);
                    expiredTime = runtime.time() + 200;
                    inTakeStep=2;
                }
                break;
             default:
                break;
        }
    }
}
