package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class WobbleGoalReset extends UltimateGoalRobot{
    @Override
    public void start() {

    }

    @Override
    public void init() {
        super.init();
        getForkliftSubsystem2().setPresetMode(false);
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();

        while(getEnhancedGamepad2().isB()){
            getForkliftSubsystem2().getForkliftMotor().setPower(-0.35);
        }
        while(getEnhancedGamepad2().isX()){
            getForkliftSubsystem2().getForkliftMotor().setPower(0.35);
        }
        getForkliftSubsystem2().getForkliftMotor().setPower(0);
        telemetry.addLine("A to go down, Y to go up");
    }

}
