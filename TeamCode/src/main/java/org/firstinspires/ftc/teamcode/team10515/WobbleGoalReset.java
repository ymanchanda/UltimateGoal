package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WobbleGoalReset extends UltimateGoalRobot{
    @Override
    public void start() {

    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();
        while(getEnhancedGamepad1().isA()){
            getForkliftSubsystem2().getForkliftMotor().setPower(-0.35);
        }
        while(getEnhancedGamepad1().isY()){
            getForkliftSubsystem2().getForkliftMotor().setPower(0.35);
        }
        getForkliftSubsystem2().getForkliftMotor().setPower(0);
        telemetry.addLine("A to go down, Y to go up");
    }

}
