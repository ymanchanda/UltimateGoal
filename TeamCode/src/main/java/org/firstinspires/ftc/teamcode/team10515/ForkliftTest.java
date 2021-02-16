package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.control.ForkliftAngle;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Wobble Goal", group = "Test")
public class ForkliftTest extends UltimateGoalRobot{
    public ForkliftAngle f;

    public double angle0 = 0;
    public double angle1 = 45;
    public double angle2 = 155;

    public RevMotor forkliftMotor;

    public ElapsedTime btnPressedA;

    public boolean nextState = false;
    public boolean goingDown = false;

    public int currentEncoderTicks = 0;
    public double power = 0;

    @Override
    public void start() {

    }

    @Override
    public void init() {
        super.init();
        forkliftMotor = getForkliftSubsystem().getForkliftMotor();
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        f = new ForkliftAngle(0.15, 0.6, 0.7);
        f.setAngle(angle0);
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();
        if(getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250){
            btnPressedA.reset();
            forkliftMotor.setPower(0);
            nextState = true;
        }

        if(nextState){
            nextState = false;
            if(f.getAngle(currentEncoderTicks) < angle1 &&
                !goingDown){
                goingDown = false;
                f.setAngle(f.getAngle(currentEncoderTicks), angle1);
                telemetry.addLine("1");
            }
            else if(f.getAngle(currentEncoderTicks) >= angle1 &&
                    f.getAngle(currentEncoderTicks) < angle2 &&
                    !goingDown){
                f.setAngle(f.getAngle(currentEncoderTicks), angle2);
                goingDown = true;
                telemetry.addLine("2");
            }
            else if(f.getAngle(currentEncoderTicks) >= angle2 && goingDown){
                f.setAngle(f.getAngle(currentEncoderTicks), angle1);
                goingDown = true;
                telemetry.addLine("3");
            }
            else if(f.getAngle(currentEncoderTicks) <= angle1 &&
                    f.getAngle(currentEncoderTicks) > angle0 &&
                    goingDown){
                f.setAngle(f.getAngle(currentEncoderTicks), angle0);
                goingDown = false;
                telemetry.addLine("4");
            }
        }

        currentEncoderTicks = forkliftMotor.getCurrentEncoderTicks();
        power = f.getSpeed(currentEncoderTicks);

        forkliftMotor.setPower(power);
        telemetry.addLine("Encoder ticks: " + currentEncoderTicks);
        telemetry.addLine("Power: " + power);
        telemetry.addLine("Going down?: " + goingDown);
        telemetry.update();

    }

}

