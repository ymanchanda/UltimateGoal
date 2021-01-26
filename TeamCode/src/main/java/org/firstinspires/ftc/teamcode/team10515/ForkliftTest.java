package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.EndGameExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStoneGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FoundationStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;

@TeleOp(name = "Wobble Goal", group = "Test")

public class ForkliftTest extends UltimateGoalRobot {
//    public double servoPos = 0;

    public double forkliftPower = 0;

    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;

    public boolean toggle = true;

    static final double COUNTS_PER_MOTOR_REV = 134.4;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 10.25 * 2; // Wobble Goal Mover Height
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*Math.PI);

    @Override
    public void start(){
//        telemetry.addData("Started", "Ready for Command");
//        telemetry.update();
    }

//    @Override
//    public void init() {
//        /* Initialize the hardware map*/
//        super.init();
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Init", "Hello Ultimate Goal Robot");    //
//        updateTelemetry(telemetry);
//
//        //setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
//        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    }
//hi
    @Override
    public void loop() {
        super.loop();

        if(reachedPosition()){
            getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if(getEnhancedGamepad1().isyLast()) {
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
        }
        else if(getEnhancedGamepad1().isaLast()) {
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
        }
        telemetry.addLine("Position" + getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks());
        telemetry.update();
    }

    public boolean reachedPosition(){
        if(getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() < 470){
            return false;
        }
        return true;
    }

}


