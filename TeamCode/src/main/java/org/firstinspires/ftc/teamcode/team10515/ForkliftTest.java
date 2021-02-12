//package org.firstinspires.ftc.teamcode.team10515;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
//
//@TeleOp(name = "Wobble Goal", group = "Test")
//
//public class ForkliftTest extends UltimateGoalRobot {
//    public static final int stopPosition = 470;
//    private static final int threshold = 50;
//    public static final int maxPosition = stopPosition + threshold;
//
//    public double forkliftPower = 0;
//
//    public long currentTime = 0;
//    public long lastTimeA = 0;
//    public long lastTimeY = 0;
//
//    public ElapsedTime btnPressedA;
//    public ElapsedTime btnPressedY;
//
//    public boolean toggleY = false;
//
//    public int lastEncoderTicks;
//    public int currentEncoderTicks = 0;
//
//    public boolean tooHigh = false;
//
//    static final double COUNTS_PER_MOTOR_REV = 134.4;
//    static final double DRIVE_GEAR_REDUCTION = 2;
//    static final double WHEEL_DIAMETER_INCHES = 10.25 * 2; // Wobble Goal Mover Height
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
//
//    @Override
//    public void start() {
//
//    }
//
//    @Override
//    public void init(){
//        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//
//        if (reachedStopPosition() && !tooHigh) {
//            getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
//        }
//
//        if (currentEncoderTicks > maxPosition) {//Check if too high
//            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
//            tooHigh = true;
//        }
//
//        if (tooHigh && currentEncoderTicks < stopPosition) {//Check if too low even after holding
//            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
//            tooHigh = false;
//        }
//
//        if (lastEncoderTicks - currentEncoderTicks > 0 &&
//                (getForkliftSubsystem().getStateMachine().getState() == ForkliftStateMachine.State.UP ||
//                        getForkliftSubsystem().getStateMachine().getState() == ForkliftStateMachine.State.IDLE)) {//Check if forklift is moving down when its not supposed to
//                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.HOLD);//Counteract weight of wobble goal
//        }
//
//        if (getEnhancedGamepad1().isyLast() && btnPressedY.milliseconds() > 250) {
//            btnPressedY.reset();
//            toggleY = !toggleY;
//            if(toggleY) {
//                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
//            }else{
//                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.HOLD);
//            }
//        } else if (getEnhancedGamepad1().isaLast()) {
//            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
//
//            if (currentEncoderTicks <= 0) {//Check if forklift has reached down position
//                //Stop motor
//                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
//            }
//        }
//
//        lastEncoderTicks = currentEncoderTicks;
//        currentEncoderTicks = getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks();
//
//        telemetry.addLine("Position" + currentEncoderTicks);
//        telemetry.addLine("Last Position" + lastEncoderTicks);
//        telemetry.update();
//    }
//
//    public boolean reachedStopPosition() {
//        if (getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() < stopPosition) {
//            return false;
//        }
//        return true;
//    }
//
//}
//
//

package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;

@TeleOp(name = "Wobble Goal", group = "Test")
//@Disabled
public class ForkliftTest extends UltimateGoalRobot{

    static final double COUNTS_PER_MOTOR_REV = 134.4;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double DEGREES_PER_ENCODER_TICK = 360/(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);
//    static final double WHEEL_DIAMETER_INCHES = 10.25 * 2; // Wobble Goal Mover Height
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        super.loop();
        getForkliftSubsystem().getForkliftMotor().setTargetPosition(getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() + (int)(90 * DEGREES_PER_ENCODER_TICK));
        getForkliftSubsystem().getForkliftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() > (int)(90/DEGREES_PER_ENCODER_TICK)) {
            getForkliftSubsystem().getForkliftMotor().setPower(0);
        }
        else{
            getForkliftSubsystem().getForkliftMotor().setPower(0.75);
        }
    }


}

