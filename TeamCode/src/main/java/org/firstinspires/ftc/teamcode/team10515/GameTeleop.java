package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeServoStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;


/**
 * This {@code class} acts as the driver-controlled program for FTC team 10515 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks -> Mecanum drive
 *          Left-trigger           -> Auto foundation movement
 *          Y-button (pressed)     -> Forklift up
 *          X-button (pressed)     -> Forklift down
 *      //Vision:
 *          //Left bumper (pressed)  -> Auto feed
 *      Right bumper (pressed) -> Toggle end game extension slides to extend
 *  User 2:
 *          Dpad-down              -> Stops Shooter
 *          Dpad-right             -> Speed for Power Shots
 *          Dpad-Up                -> Speed for high shots
 *          Right-trigger          -> Turns the Intake On
 *          Left - Trigger         -> Reverses Intake
 *          Back                   -> Stopping the intake
 *          Y - Button             -> Elevator goes up
 *          A - Button             -> Elevator goes down
 *          X - Button             -> Flicker hit rings and come back
 *          Left bumper (pressed)  -> Toggles automation
 *          Right bumper (pressed) -> Intake Servo Hit
 *
 * @see UltimateGoalRobot
 */
@TeleOp(name = "Game Tele-Op", group = "Main")
public class GameTeleop extends UltimateGoalRobot {
    private boolean iselevatorUp = false;   //elevator starts in down position
    private boolean isFlicked = false;      //flickers are inside
    private boolean isAuto = false;          //robot is running manual
    private boolean confirmElevatorUp = false;      //confirm elevator is up
    private boolean confirmElevatorDown = false;    //confirm elevator is down
    private boolean intakeServo = false;            //intake Servo activated
    private int count = 0;
    //no rings down position = 9.2  //23.6CM
    //3 rings down position = 8.5   //22CM
    //3 rings up position = 2.65
    //no rings up position = 4.5
    public double upThreshold = 8.65;
    public double downThreshold = 4.45;

    public ElapsedTime resetFlicker = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //public ElapsedTime btnPressedRightBumper = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime doubleCheckUp = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime doubleCheckDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //Wobble goal Forklift
    public static final int alignPosition = 650;
    public static final int topPosition = 2220;
    public int lastEncoderTicks;
    public int currentEncoderTicks = 0;
    public boolean pastAlign, pastTop = false;

    @Override
    public void start() {
        //Feeder.setManualControlExtension(() -> gamepad2.b ? 0.5d : gamepad2.x ? -0.5d : 0d);
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
        setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, new Rotation2d(gamepad1.right_stick_x, false)));

        //Gamepad2 Update flywheel intake - In:Right Trigger, Out:Left Trigger, Stop: Back
        if(getEnhancedGamepad2().getRight_trigger()>0) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
        } else if(getEnhancedGamepad2().getLeft_trigger() > 0) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.OUTTAKE);
        } else if(getEnhancedGamepad2().isBack()) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
        }

        //Gamepad2 Intake Servo - Right Bumper
//        if(getEnhancedGamepad2().isRightBumperLast()){
//            getIntakeServoSubsystem().getStateMachine().updateState(IntakeServoStateMachine.State.HIT_RING);
//            btnPressedRightBumper.reset();
//            intakeServo = true;
//        }
//        if (btnPressedRightBumper.milliseconds()>700 && intakeServo){
//            getIntakeServoSubsystem().getStateMachine().updateState(IntakeServoStateMachine.State.STANDBY);
//            intakeServo = false;
//        }

        //Toggle Shooter
        if(getEnhancedGamepad2().isDpad_up()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
        }

        else if(getEnhancedGamepad2().isDpad_right()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
        }
        else if(getEnhancedGamepad2().isDpad_left()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED3);
        }
        else if(getEnhancedGamepad2().isDpad_down()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        }

        if(getEnhancedGamepad2().isRightBumperLast()){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
            resetFlicker.reset();
            isFlicked = true;
        }
        if (isFlicked && resetFlicker.milliseconds()>100){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);

        }

        //Auto, Elevator is up, check if we should move down, confirmation
        if(iselevatorUp && elevatorSensor.getDistance(DistanceUnit.INCH) > downThreshold && !confirmElevatorDown){
            confirmElevatorDown = true;
            doubleCheckDown.reset();
        }

        if(confirmElevatorDown && doubleCheckDown.milliseconds() > 1000) {
            if (iselevatorUp && elevatorSensor.getDistance(DistanceUnit.INCH) > downThreshold ) {
                iselevatorUp = false;
                if (isAuto) {
                    getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                    getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                    getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                }
            }
            confirmElevatorDown = false;
        }

        //Auto, Elevator is down, check if we should move up, confirmation
        if(!iselevatorUp && elevatorSensor.getDistance(DistanceUnit.INCH) < upThreshold && !confirmElevatorUp){
            confirmElevatorUp = true;
            doubleCheckUp.reset();
        }

        if(confirmElevatorUp && doubleCheckUp.milliseconds()>1000){
            if((!iselevatorUp) && elevatorSensor.getDistance(DistanceUnit.INCH) < upThreshold) {
                iselevatorUp = true;
                if (isAuto) {
                    getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                    getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
                    getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                }
            }
            confirmElevatorUp = false;
        }

        //Gamepad2 Manually move Elevator up && btnPressedA.milliseconds()>1250  && btnPressedA.milliseconds()>1250
        if(!isAuto && getEnhancedGamepad2().isyLast()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
            iselevatorUp = true;    //Elevator Moved Up and shooter starts
        } else if(!isAuto && getEnhancedGamepad2().isaLast()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
            iselevatorUp = false;   //Elevator Moved Down
        }

        //WobbleGoal processing
        WobbleGoalv2();

        if (getEnhancedGamepad2().isLeftBumperLast()){
            isAuto = !isAuto;
        }

        telemetry.addLine("Auto Mode: " + isAuto);
        telemetry.addLine("Wobble Goal: " + getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks());
        telemetry.addLine("Past Align: " +pastAlign);
        telemetry.addLine("Intake Output: " + getIntakeMotorSubsystem().getOutput());
        telemetry.addLine("Shooter Output: " + getShooterSubsystem().getOutput());
        telemetry.update();
    }

    void WobbleGoalv2()
    {
        //brake 1st time when it reaches align
        if (reachedUpPosition(alignPosition) && !pastAlign) {
            getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastAlign = true;
        }

        //brake if it was past align and went down past align + 20
        if (pastAlign && reachedDownPosition(alignPosition+20)) {
            getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastAlign = false;      //reset pastAlign to false as it's down
        }

        if (reachedUpPosition(topPosition) && !pastTop) {
            getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastTop = true;
        }

        if (reachedDownPosition(topPosition)) {
            pastTop = false;
        }

        if (reachedDownPosition(50)) {
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if (getEnhancedGamepad2().isbLast()) {
            if (!reachedUpPosition(topPosition))
                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);

        } else if (getEnhancedGamepad2().isxLast()) {
            if (!reachedDownPosition(0)) {
                getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
            }
        }

//        lastEncoderTicks = currentEncoderTicks;
//        currentEncoderTicks = getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks();

    }

    public boolean reachedUpPosition(double position) {
        if (getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() < position)
            return false;
        else
        return true;
    }

    public boolean reachedDownPosition(double position) {
        if (getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() > position)
            return false;
        else
            return true;
    }

}
