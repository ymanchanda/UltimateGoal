package org.firstinspires.ftc.teamcode.team10515;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.team10515.auto.UGBase;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
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
@TeleOp(name = "Auto Tele-Op", group = "Main")
public class autoTeleop extends UGTeleOpRobot {
    private boolean iselevatorUp = false;   //elevator starts in down position
    private boolean isFlicked = false;      //flickers are inside
    private boolean isAuto = false;          //robot is running manual
    private boolean confirmElevatorUp = false;      //confirm elevator is up
    private boolean confirmElevatorDown = false;    //confirm elevator is down
    private boolean intakeServo = false;            //intake Servo activated
    private int count = 0;
    public double upThreshold = 8.65;
    public double downThreshold = 4.45;
    private int intakeChange = 0;

    enum FlickState {
        FLICK,
        WAITFLICK,
        BACK,
        WAITBACK,
        IDLE
    }
    int NumFlicks = 0;
    FlickState FlickThree = FlickState.IDLE;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    public ElapsedTime resetFlicker = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime resetFlickThree = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //Wobble goal Forklift
    public static final int alignPosition = 650;
    public static final int topPosition = 2220;
    public boolean pastAlign, pastTop = false;
    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d highShotVector = new Vector2d(2, 40);
    Vector2d wobbleGoalDepositVector = new Vector2d(-60, 48);
    // The heading we want the bot to end on for targetA
    double highShotHeading = Math.toRadians(10);

    @Override
    public void init() {
        drive = new UGBase(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
        super.init();
    }

    @Override
    public void start() {
        //Feeder.setManualControlExtension(() -> gamepad2.b ? 0.5d : gamepad2.x ? -0.5d : 0d);
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();

//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x
//                )
//        );
      Pose2d poseEstimate = drive.getPoseEstimate();
        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                if (getEnhancedGamepad1().isaLast()) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    Trajectory highShotPosition = drive.trajectoryBuilder(poseEstimate)
                            .splineToConstantHeading(highShotVector, highShotHeading)
                            .build();

                    drive.followTrajectoryAsync(highShotPosition);
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                if(getEnhancedGamepad1().isyLast()){
                    Trajectory wobbleGoalPosition = drive.trajectoryBuilder(poseEstimate)
                            .splineTo(wobbleGoalDepositVector, Math.toRadians(270))
                            .build();
                    drive.followTrajectoryAsync(wobbleGoalPosition);
                    drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
                    currentMode = Mode.AUTOMATIC_CONTROL;
                } //hi
                break;
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (gamepad1.x) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }

        //Gamepad2 Update flywheel intake - In:Right Trigger, Out:Left Trigger, Stop: Back
        if(getEnhancedGamepad2().getRight_trigger()>0) {
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
        } else if(getEnhancedGamepad2().getLeft_trigger() > 0) {
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.OUTTAKE);
        } else if(getEnhancedGamepad2().isBack()) {
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
        }

        if (getEnhancedGamepad1().isaLast()) {
            intakeChange += 1;
            if (intakeChange > 4) intakeChange = 0;
            switch(intakeChange) {

                case 1:
                    drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE1);
                    break;
                case 2:
                    drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE2);
                    break;
                case 3:
                    drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE3);
                    break;
                case 4:
                    drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE4);
                    break;
                case 0:
                    drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                    break;
            }
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
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
        }

        else if(getEnhancedGamepad2().isDpad_right()) {
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
        }
        else if(getEnhancedGamepad2().isDpad_left()) {
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED3);
        }
        else if(getEnhancedGamepad2().isDpad_down()) {
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        }

        if(getEnhancedGamepad2().isRightBumperLast()){
            drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
            resetFlicker.reset();
            isFlicked = true;
        }
        if (isFlicked && resetFlicker.milliseconds()>100){
            drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        }

        //Gamepad2 Manually move Elevator up && btnPressedA.milliseconds()>1250  && btnPressedA.milliseconds()>1250
        if(!isAuto && getEnhancedGamepad2().isyLast()) {
            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
            iselevatorUp = true;    //Elevator Moved Up and shooter starts
        } else if(!isAuto && getEnhancedGamepad2().isaLast()) {
            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
            iselevatorUp = false;   //Elevator Moved Down
        }

        //WobbleGoal processing
        WobbleGoalv2();

        if (getEnhancedGamepad2().isLeftBumperLast()){
            FlickThree = FlickState.FLICK;
        }

        FlickThree();

        telemetry.addLine("Pose"+poseEstimate.getX()+", "+poseEstimate.getY()+", "+poseEstimate.getHeading());
        telemetry.addLine("Auto Mode: " + isAuto);
        telemetry.addLine("Wobble Goal: " + drive.robot.getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks());
        telemetry.addLine("Past Align: " +pastAlign);
        telemetry.addLine("Intake Output: " + drive.robot.getIntakeMotorSubsystem().getOutput());
        telemetry.addLine("Shooter Output: " + drive.robot.getShooterSubsystem().getOutput());
        telemetry.update();
    }

    void FlickThree()
    {
        switch (FlickThree) {
            case IDLE:
                NumFlicks = 0;
                break;
            case FLICK:
                drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                FlickThree = FlickState.WAITFLICK;
                resetFlickThree.reset();
                break;
            case WAITFLICK:
                if (resetFlickThree.milliseconds() > 100) {
                    FlickThree = FlickState.BACK;
                }
                break;
            case BACK:
                drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
                FlickThree = FlickState.WAITBACK;
                resetFlickThree.reset();
                break;
            case WAITBACK:
                if (resetFlickThree.milliseconds() > 300) {
                    if (NumFlicks < 2) {
                        FlickThree = FlickState.FLICK;
                    }
                    else {
                        FlickThree = FlickState.IDLE;
                    }
                    NumFlicks++;
                }
                break;
        }
    }

    void WobbleGoalv2()
    {
        //brake 1st time when it reaches align
        if (reachedUpPosition(alignPosition) && !pastAlign) {
            drive.robot.getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastAlign = true;
        }

        //brake if it was past align and went down past align + 20
        if (pastAlign && reachedDownPosition(alignPosition + 20)) {
            drive.robot.getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastAlign = false;      //reset pastAlign to false as it's down
        }

        if (reachedUpPosition(topPosition) && !pastTop) {
            drive.robot.getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
            pastTop = true;
        }

        if (reachedDownPosition(topPosition)) {
            pastTop = false;
        }

        if (reachedDownPosition(50)) {
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if (getEnhancedGamepad2().isbLast()) {
            if (!reachedUpPosition(topPosition))
                drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);

        } else if (getEnhancedGamepad2().isxLast()) {
            if (!reachedDownPosition(0)) {
                drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
            }
        }

//        lastEncoderTicks = currentEncoderTicks;
//        currentEncoderTicks = getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks();

    }

    public boolean reachedUpPosition(double position) {
        if (drive.robot.getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() < position)
            return false;
        else
        return true;
    }

    public boolean reachedDownPosition(double position) {
        if (drive.robot.getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks() > position)
            return false;
        else
            return true;
    }

}
