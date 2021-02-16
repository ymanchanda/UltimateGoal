package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.PoseStorage;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "Test Odo", group = "drive")
public class TestOdo extends LinearOpMode {
    UGBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;
    boolean shooterRunning = false;
    boolean elevatorUp = false;
    boolean flickerchange = false;
    boolean goDown, goUp = false;
    public static final int topPosition = 2020;
    public static final int alignPosition = 1000;
    enum WobbleState {
        ZERO,
        ALIGN,
        TOP
    }

    WobbleState wobbleTo = WobbleState.ZERO;
    ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WOBBLE,
        WAIT0,
        TRAJ1, //Move to the left most powershot
        TRAJ2, //Staffe to the middle
        TRAJ3, //Straffe to the left
        TRAJ4, //go to the wobble goal
        PARK, //park if no rings or 4 rings
        SHOOTRING, //intake and shoot 1 ring in high goal
        IDLE, //do nothing
        WAIT1, //Wait before shooting
        WAIT2, //Wait before shooting
        WAIT3, //Wait before shooting
        GOTOZONE,
        WAIT4,
        wobble2,
        WAIT5,
        GETRINGS,
        INTAKE,
        HIGHSHOT,
        WAIT6,
        WOBBLEDOWN
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-60, 19, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new UGBase(hardwareMap);
        drive.setPoseEstimate(startPose);

        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(2, 12),Math.toRadians(10))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(8)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(8)
                .build();
        Trajectory zoneA = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(2,50),Math.toRadians(0))
                .build();
        Trajectory zoneB = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(29,26),Math.toRadians(0))
                .build();
        Trajectory zoneC = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(47,54),Math.toRadians(0))
                .build();
        Trajectory parkb = drive.trajectoryBuilder(zoneB.end())
                .back(24)
                .build();
        Trajectory parkC = drive.trajectoryBuilder(zoneC.end())
                .back(36)
                .build();
        Trajectory release = drive.trajectoryBuilder(zoneA.end())
                .back(1)
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(zoneB.end(),true)
                .splineTo(new Vector2d(-60,19),Math.toRadians(180))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(wobble2.end())
                .strafeLeft(18)
                .build();
        Trajectory forward = drive.trajectoryBuilder(strafe.end())
                .forward(40)
                .build();
        Trajectory shootRing = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(2,48),Math.toRadians(10))
                .build();
        waitForStart();

        waitTimer.reset();
        UGCV.numRings numRingsDetected = drive.getRingsUsingImage(false);
        numRingsDetected = drive.getRingsUsingImage(false);
        telemetry.addLine("Num Rings: "+ numRingsDetected);
        double CVTime = waitTimer.milliseconds();
        telemetry.addLine("Milliseconds:" + CVTime);
        telemetry.update();

        if (isStopRequested()) return;

        //drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTOUP);
        //drive.robot.getForkliftSubsystem().update(getDt());
        //wobbleTo = WobbleState.TOP;
        currentState = State.WOBBLE;

        while (opModeIsActive() && !isStopRequested()) {
//            if (!shooterRunning) {
//                drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
//                shooterRunning = true;
//            }
//
//            if (!elevatorUp) {
//                drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
//                elevatorUp = true;
//            }
//
//            if (flickerchange && flickerTime.milliseconds() > 100) {
//                drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
//                flickerchange = false;
//            }

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {
                case WOBBLE:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT0;
                        waitTimer.reset();
                    }
                    break;
                case WAIT0:
                    if (waitTimer.milliseconds() >= 3000) {
                        //currentState = State.WOBBLEDOWN;
                        //drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTODOWN);
                        //drive.robot.getForkliftSubsystem().update(getDt());
                        //wobbleTo = WobbleState.ALIGN;
                        currentState = State.TRAJ1;
                        drive.followTrajectoryAsync(traj1);
//                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
//                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
                    }
                    break;
                case WOBBLEDOWN:
                    if (waitTimer.milliseconds() >= 100) {
                        currentState = State.IDLE;
                    }
                    break;
                case TRAJ1:
                    if (!drive.isBusy()) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAIT1;
                        waitTimer.reset();
                    }
                    break;
                case WAIT1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer.milliseconds() >= 500) {
                        currentState = State.TRAJ2;
                        drive.followTrajectoryAsync(traj2);
                    }
                    break;
                case TRAJ2:
                    if (!drive.isBusy()) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAIT2;
                        waitTimer.reset();
                    }
                    break;
                case WAIT2:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer.milliseconds() >= 500) {
                        currentState = State.TRAJ3;
                        drive.followTrajectoryAsync(traj3);
                    }
                    break;
                case TRAJ3:
                    if (!drive.isBusy()) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        waitTimer.reset();
                        currentState = State.WAIT3;
                        //drive.followTrajectoryAsync(traj2);
                    }
                    break;
                case WAIT3:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer.milliseconds() >= 1000) {
                        if(numRingsDetected == UGCV.numRings.ZERO){
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneA);

                        }
                        else if(numRingsDetected == UGCV.numRings.ONE){
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneB);
                        }
                        else{
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneC);
                        }
                    }
                    break;
                case GOTOZONE:
                    if (!drive.isBusy()) {
                        //drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTODOWN);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        goDown = true;
//                        if (elevatorUp) {
//                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
//                            elevatorUp = false;
//                        }
//                        if (shooterRunning) {
//                            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
//                            shooterRunning = false;
//                        }
                        waitTimer.reset();
//                        if (numRings == UGCV.numRings.ZERO){
//                            currentState = State.WAIT5;
//                        }
//                        else
                        currentState = State.WAIT4;
                    }
                    break;
                case WAIT4:
                    if (waitTimer.milliseconds() >= 2000) {
                        if (numRingsDetected == UGCV.numRings.ONE) {
                            currentState = State.wobble2;
                            drive.followTrajectoryAsync(wobble2);
                        }
                        else if(numRingsDetected == UGCV.numRings.FOUR){
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(parkC);
                        }
                        else{
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(release);
                        }
                    }
                    break;
                case wobble2:
                    if (!drive.isBusy()) {
                        //drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                        //drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        currentState = State.WAIT5;
                        waitTimer.reset();
                    }
                    break;
                case WAIT5:
                    if(waitTimer.milliseconds() >= 500) {
                        currentState = State.GETRINGS;
                        drive.followTrajectoryAsync(strafe);
                    }
                    break;
                case GETRINGS:
                    if (!drive.isBusy()){
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        currentState = State.INTAKE;
                        drive.followTrajectoryAsync(forward);
                    }
                    break;
                case INTAKE:
                    if(!drive.isBusy()){
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                        drive.followTrajectoryAsync(shootRing);
                        currentState = State.HIGHSHOT;

                    }
                    break;
                case HIGHSHOT:
                    if(!drive.isBusy()){
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        currentState = State.WAIT6;
                        waitTimer.reset();
                    }
                    break;
                case WAIT6:
                    if (waitTimer.milliseconds() >= 1000) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    if (elevatorUp) {
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        elevatorUp = false;
                    }
                    if (shooterRunning) {
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                        shooterRunning = false;
                    }
                    break;
            } //end for switch

            drive.update();
            drive.getExpansionHubs().update(getDt());
            drive.robot.getShooterSubsystem().update(getDt());
            drive.robot.getFlickerSubsystem().update(getDt());
            drive.robot.getForkliftSubsystem().update(getDt());
            drive.robot.getPulleySubsystem().update(getDt());
            drive.robot.getIntakeMotorSubsystem().update(getDt());
            WobbleGoal();

            telemetry.addLine("# Rings"+numRingsDetected);
            telemetry.addLine("Milliseconds:" + CVTime);
            telemetry.addLine("Output"+drive.robot.getShooterSubsystem().getOutput());
            telemetry.addLine("Speed"+drive.robot.getShooterSubsystem().getState().getSpeed());
            telemetry.addLine("Velocity"+drive.robot.getShooterSubsystem().getShooterWheel1().getVelocity());
            telemetry.addLine("Elevator up"+elevatorUp);
            telemetry.addLine("Pose Estimate:"+drive.getPoseEstimate().getX()+","+drive.getPoseEstimate().getY()+", "+drive.getPoseEstimate().getHeading() );

            telemetry.update();
        } //end of while
        drive.setMotorPowers(0.0,0.0,0.0,0.0);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public static TimeProfiler getUpdateRuntime() {return updateRuntime;}
    public static void setUpdateRuntime(TimeProfiler updaRuntime) { updateRuntime = updaRuntime; }
    public static double getDt() { return dt;}
    public static void setDt(double pdt) { dt = pdt; }

    void WobbleGoal()
    {
        switch (wobbleTo) {
            case ZERO:
                if (reachedDownPosition(50)) {
                    drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
                }
                break;
            case TOP:
                if (reachedUpPosition(topPosition)) {
                    drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
                }
                break;
            case ALIGN:
                if (reachedDownPosition(alignPosition)) {
                    drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
                }
                break;
        }
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
