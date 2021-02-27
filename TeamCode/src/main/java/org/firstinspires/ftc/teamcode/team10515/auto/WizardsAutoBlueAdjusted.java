package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.PoseStorage;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "Wizards Auto Blue Adjusted", group = "drive")
public class WizardsAutoBlueAdjusted extends LinearOpMode {
    UGBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;
    boolean shooterRunning = false;
    boolean elevatorUp = false;
    boolean forkliftUp = false;
    boolean flickerchange = false;
    int shotcount = 0;
    boolean goDown = false;
    public int lastEncoderTicks;
    public int currentEncoderTicks = 0;
    //    public static final int topPosition = 430;
    public static final int maxPosition = 2020; //max position
    //    public static final int topPosition2 = 2020;
    public static final int alignPosition = 1000;
    int flickerWaitTime = 400;

    enum WobbleState {
        ZERO,
        ALIGN,
        TOP
    }

    ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        STRAFE,
        WAIT0,
        TRAJ1, //Move to the left most powershot
        TRAJ2, //Staffe to the middle
        TRAJ3, //Straffe to the left
        TRAJ4, //go to the wobble goal
        PARK, //park if no rings or 4 rings
        SHOOTRING, //intake and shoot 1 ring in high goal
        IDLE,
        WAIT1,
        WAIT2,
        WAIT3,//do nothing
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-62.375, 17, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new UGBase(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.closeCamera();


        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);

        Trajectory initialstrafe = drive.trajectoryBuilder(startPose)
                .strafeRight(7)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(initialstrafe.end())
                .splineTo(new Vector2d(2, 18), Math.toRadians(-7))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(7)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(7)
                .build();
        Trajectory park = drive.trajectoryBuilder(traj1.end())
                .forward(10)
                .build();
        Trajectory finish = drive.trajectoryBuilder(park.end())
                .strafeRight(5)
                .build();

        waitForStart();
        //UGCV.numRings numRings = drive.getRingsUsingImage(false);
        //telemetry.addLine("Num Rings: " + numRings);
        //telemetry.update();

        if (isStopRequested()) return;

        currentState = State.STRAFE;

        while (opModeIsActive() && !isStopRequested()) {
            if (!shooterRunning) {
                drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.POLESHOT);
                shooterRunning = true;
            }

            if (!elevatorUp) {
                drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                elevatorUp = true;
            }

            if (flickerchange && flickerTime.milliseconds() > 100) {
                drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
                flickerchange = false;
            }

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {
                case STRAFE:
                    if (waitTimer.milliseconds() >= 100) {
                        currentState = State.WAIT0;
                        drive.followTrajectoryAsync(initialstrafe);
                        waitTimer.reset();
                    }

                        break;
                case WAIT0:
                    if (waitTimer.milliseconds() >= 200) {
                        currentState = State.TRAJ1;
                        drive.followTrajectoryAsync(traj1);
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
                    if (waitTimer.milliseconds() >= 400) {
                        currentState = State.TRAJ2;
                        drive.turn(Math.toRadians(10));
                        waitTimer.reset();
                        //drive.followTrajectoryAsync(traj2);
                    }
                    break;
                case TRAJ2:
                    if (!drive.isBusy() && waitTimer.milliseconds() > flickerWaitTime){
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
                    if (waitTimer.milliseconds() >= 400) {
                        currentState = State.TRAJ3;
                        drive.turn(Math.toRadians(8));
                        waitTimer.reset();
                        //drive.followTrajectoryAsync(traj3);
                    }
                    break;
                case TRAJ3:
                    if (!drive.isBusy() && waitTimer.milliseconds() > flickerWaitTime) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        waitTimer.reset();
                        currentState = State.WAIT3;
                    }
                    break;
                case WAIT3:
                    if (waitTimer.milliseconds() >= 500) {
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(park);
//                        if (numRings == UGCV.numRings.ZERO) {
//                            currentState = State.PARK;
//                            drive.followTrajectoryAsync(park);
//
//                        } else if (numRings == UGCV.numRings.ONE) {
//                            currentState = State.PARK;
//                            drive.followTrajectoryAsync(park);
//                        } else {
//                            currentState = State.PARK;
//                            drive.followTrajectoryAsync(park);
//                        }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(finish);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    if (elevatorUp) {
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                        elevatorUp = false;
                    }
                    if (shooterRunning) {
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                        shooterRunning = false;
                    }
                    break;
            }

            drive.update();
            drive.getExpansionHubs().update(getDt());
            drive.robot.getShooterSubsystem().update(getDt());
            drive.robot.getFlickerSubsystem().update(getDt());
            //drive.robot.getForkliftSubsystem().update(getDt());
            drive.robot.getPulleySubsystem().update(getDt());
            drive.robot.getIntakeMotorSubsystem().update(getDt());
            //WobbleGoal();

            telemetry.addLine("Output" + drive.robot.getShooterSubsystem().getOutput());
            telemetry.addLine("Speed" + drive.robot.getShooterSubsystem().getState().getSpeed());
            telemetry.addLine("Velocity" + drive.robot.getShooterSubsystem().getShooterWheel1().getVelocity());
            telemetry.addLine("Elevator up" + elevatorUp);
            telemetry.addLine("Shot count" + shotcount);


            telemetry.update();
        } // end of while
        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }

}