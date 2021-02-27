package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.PoseStorage;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;
import org.firstinspires.ftc.teamcode.team10515.states.GripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "XV Blue Auto", group = "drive")
public class XVBlueAuto extends LinearOpMode {
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
    enum WobbleState {
        ZERO,
        ALIGN,
        TOP
    }

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
        LETGOWOBBLE,
        WAIT4,
        RINGDETECTION,
        BLINDFOLLOW,
        RETURN,
        SHOOT,
        WAIT9,
        WAIT10,
        MIDPOINT,
        WAIT7,
        wobble2,
        WAIT5,
        GRABWOBBLE2,
        HOLDWOBBLE,
        GETRINGS,
        WAIT8,
        STRAFETOWOBBLE,
        INTAKE,
        RING1,
        WAITR1,
        WAITR2,
        RING2,
        RING3,
        WAITTOGOUP,
        WAITSHOT1,
        WAITSHOT2,
        WAITSHOT3,
        WAITFINAL,
        HIGHSHOT,
        PLACEWOBBLE,
        ENDWOBBLE,
        WAIT6,
        FINISH,

    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-62.375, 17, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new UGBase(hardwareMap);
        drive.setPoseEstimate(startPose);

        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.INIT);
        drive.robot.getGripperSubsystem().update(getDt());
        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(2.5, 12), Math.toRadians(0))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(8)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(8)
                .build();
        Trajectory zoneA = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(12, 40), Math.toRadians(0))
                .build();
        Trajectory zoneB = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(40, 26), Math.toRadians(0))
                .build();
        Trajectory zoneC = drive.trajectoryBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(60, 45), Math.toRadians(0))
                .build();
        Trajectory parkC = drive.trajectoryBuilder(zoneC.end())
                .back(36)
                .build();
        Trajectory release = drive.trajectoryBuilder(zoneA.end())
                .back(1)
                .build();
        Trajectory wobble2 = drive.trajectoryBuilder(zoneB.end(), true)
                .splineTo(new Vector2d(-49, 19), Math.toRadians(180))//-60
                .build();
        Trajectory midpoint = drive.trajectoryBuilder(zoneC.end(), true)
                .splineTo(new Vector2d(-12, 19), Math.toRadians(180))
                .build();
        Trajectory wobble2C = drive.trajectoryBuilder(midpoint.end(), true)
                .splineTo(new Vector2d(-49, 19), Math.toRadians(180))
                .build();
        Trajectory wobble2a = drive.trajectoryBuilder(zoneA.end(), true)
                .splineTo(new Vector2d(-49, 19), Math.toRadians(180))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(wobble2.end())
                .strafeLeft(22)
                .build();
        Trajectory strafec = drive.trajectoryBuilder(wobble2C.end())
                .strafeLeft(15)
                .build();
        Trajectory forward = drive.trajectoryBuilder(strafe.end())
                .forward(40)
                .build();
        Trajectory forwardc = drive.trajectoryBuilder(strafec.end())
                .forward(55)
                .build();
        Trajectory back = drive.trajectoryBuilder(forwardc.end())
                .back(20)
                .build();
        Trajectory strafeToWobble = drive.trajectoryBuilder(back.end())
                .strafeLeft(10)
                .build();
        Trajectory shootRing = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(2, 46), Math.toRadians(0))
                .build();
        Trajectory backtoDropWobble = drive.trajectoryBuilder(strafe.end())
                .splineTo(new Vector2d(18, 40), Math.toRadians(0))
                .build();
        Trajectory strafeToDrop = drive.trajectoryBuilder(shootRing.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .strafeLeft(12)
                .build();
        Trajectory ringdetection = drive.trajectoryBuilder(release.end())
                .splineTo(new Vector2d(54, 60), Math.toRadians(-70))
                .build();
        Trajectory blindforward = drive.trajectoryBuilder(ringdetection.end())
                .forward(55)
                .build();
        Trajectory parkb = drive.trajectoryBuilder(shootRing.end())
                .forward(10)
                .build();
        Trajectory returntoshotpos = drive.trajectoryBuilder(blindforward.end())
                .splineTo(new Vector2d(-4, 29), Math.toRadians(35))
                .build();
        Trajectory goToPark = drive.trajectoryBuilder(returntoshotpos.end())
                .forward(10)
                .build();
        Trajectory ring1 = drive.trajectoryBuilder(back.end())
                .forward(10, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory ring2 = drive.trajectoryBuilder(ring1.end())
                .forward(6,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory ring3 = drive.trajectoryBuilder(ring2.end())
                .forward(6, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory end = drive.trajectoryBuilder(ring3.end())
                .forward(12)
                .build();
        Trajectory PARK = drive.trajectoryBuilder(end.end())
                .forward(10)
                .build();
        waitForStart();

        //UGCV.numRings numRings = drive.getRingsUsingImage(false);
        UGCV.numRings numRings = drive.getRingsUsingImageandBlueCam(false);
        telemetry.addLine("Num Rings: " + numRings);
        telemetry.update();

        if (isStopRequested()) return;

        currentState = State.WOBBLE;
        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
        drive.robot.getForkliftSubsystem2().update(getDt());

        //drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTOUP);
        //drive.robot.getForkliftSubsystem().update(getDt());

        //currentState = State.TRAJ1;
        //drive.followTrajectoryAsync(traj1);

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
                case WOBBLE:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT0;
                        drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.GRIP);
                        waitTimer.reset();
                    }
                    break;
                case WAIT0:
                    if (waitTimer.milliseconds() >= 600) {
                        currentState = State.TRAJ1;
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.TOP);
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
                    if (waitTimer.milliseconds() >= 200) {
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
                    if (waitTimer.milliseconds() >= 200) {
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
                    if (waitTimer.milliseconds() >= 500) {
                        if (numRings == UGCV.numRings.ZERO) {
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneA);

                        } else if (numRings == UGCV.numRings.ONE) {
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneB);
                        } else {
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneC);
                        }
                    }
                    break;
                case GOTOZONE:
                    if (!drive.isBusy()) {
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        goDown = true;
                        waitTimer.reset();
                        currentState = State.LETGOWOBBLE;
                    }
                    break;
                case LETGOWOBBLE:
                    if (waitTimer.milliseconds() >= 600) {
                        drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.INIT);
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
                        currentState = State.WAIT4;
                        waitTimer.reset();
                    }
                    break;
                case WAIT4:
                    if (waitTimer.milliseconds() >= 600) {
                        if (numRings == UGCV.numRings.ONE) {
                            currentState = State.wobble2;
                            drive.followTrajectoryAsync(wobble2);
                        } else if (numRings == UGCV.numRings.FOUR) {
                            currentState = State.MIDPOINT;
                            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                            drive.followTrajectoryAsync(midpoint);
                        } else {
                            currentState = State.wobble2;
                            drive.followTrajectoryAsync(wobble2a);
                        }
                    }
                    break;
                case RINGDETECTION:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ringdetection);
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);

                        currentState = State.BLINDFOLLOW;
                    }
                    break;
                case BLINDFOLLOW:
                    if (!drive.isBusy()) {
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE4);
                        drive.followTrajectoryAsync(blindforward);
                        currentState = State.RETURN;
                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(returntoshotpos);
                        currentState = State.SHOOT;
                    }
                    break;
                case SHOOT:
                    if (!drive.isBusy()) {
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.HIGHGOAL);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                        currentState = State.WAIT9;
                        waitTimer.reset();
                    }
                    break;
                case WAIT9:
                    if (waitTimer.milliseconds() >= 900) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        shotcount++;
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAIT10;
                        waitTimer.reset();
                    }
                    break;
                case WAIT10:
                    if (waitTimer.milliseconds() >= 200) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
                        flickerchange = true;
                        flickerTime.reset();
                        if (shotcount > 3) {
                            currentState = State.PARK;
                            shotcount = 0;
                        }
                        else{
                            currentState = State.WAIT9;
                        }
                        waitTimer.reset();
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(goToPark);
                        currentState = State.IDLE;
                    }
                    break;
                case MIDPOINT:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT7;
                        waitTimer.reset();
                    }
                    break;
                case WAIT7:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer.milliseconds() >= 100) {
                        currentState = State.wobble2;
                        drive.followTrajectoryAsync(wobble2C);
                    }
                    break;
                case wobble2:
                    if (!drive.isBusy()) {
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                        currentState = State.WAIT5;
                        waitTimer.reset();
                    }
                    break;
                case WAIT5:
                    if (waitTimer.milliseconds() >= 400) {
                        if(numRings == UGCV.numRings.ONE || numRings == UGCV.numRings.ZERO ) {
                            drive.followTrajectoryAsync(strafe);
                            currentState = State.GRABWOBBLE2;
                        }
                        else{
                            drive.followTrajectoryAsync(strafec);
                            currentState = State.GETRINGS;
                        }

                    }
                    break;
                case GRABWOBBLE2:
                    if(!drive.isBusy()){
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
                        currentState = State.HOLDWOBBLE;
                        waitTimer.reset();
                    }
                    break;
                case HOLDWOBBLE:
                    if (waitTimer.milliseconds() >= 400) {
                        if (numRings == UGCV.numRings.ONE) {
                            drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.GRIP);
                            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                            currentState = State.GETRINGS;
                            waitTimer.reset();
                        }
                        else if (numRings == UGCV.numRings.ZERO){
                            drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.GRIP);
                            currentState = State.GETRINGS;
                            waitTimer.reset();
                        }
                        else {
                            drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.GRIP);
                            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                            currentState = State.IDLE;
                            waitTimer.reset();
                        }
                    }
                    break;
                case GETRINGS:
                    if(waitTimer.milliseconds() >= 500){
                        if (numRings == UGCV.numRings.ONE) {
                            drive.followTrajectoryAsync(forward);
                            currentState = State.INTAKE;
                        }
                        else if(numRings == UGCV.numRings.ZERO){
                            drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.TOP);
                            drive.followTrajectoryAsync(backtoDropWobble);
                            currentState = State.PLACEWOBBLE;
                        }
                        else {
                            drive.followTrajectoryAsync(forwardc);
                            currentState = State.WAIT8;
                            waitTimer.reset();
                        }
                    }
                    break;
                case WAIT8:
                    if (!drive.isBusy()&& waitTimer.milliseconds() >= 500) {
                        currentState = State.STRAFETOWOBBLE;
                        drive.followTrajectoryAsync(back);

                    }
                    break;
                case STRAFETOWOBBLE:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(strafeToWobble);
                        currentState = State.GRABWOBBLE2;
                    }
                    break;
                case RING1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ring1);
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE4);
                        currentState = State.WAITR1;
                        waitTimer.reset();
                    }
                    break;
                case WAITR1:
                    if (waitTimer.milliseconds() >= 1000) {
                        currentState = State.RING2;
                        drive.followTrajectoryAsync(ring2);

                    }
                    break;
                case RING2:
                    if (!drive.isBusy()) {
                        currentState = State.WAITR2;
                        waitTimer.reset();
                    }
                    break;
                case WAITR2:
                    if (waitTimer.milliseconds() >= 1000) {
                        currentState = State.RING3;
                        drive.followTrajectoryAsync(ring3);

                    }
                    break;
                case RING3:
                    if (!drive.isBusy()) {
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.HIGHGOAL);
                        drive.turn(Math.toRadians(-345));
                        drive.followTrajectoryAsync(end);
                        waitTimer.reset();
                        currentState = State.WAITTOGOUP;
                    }
                    break;
                case WAITTOGOUP:
                    if(waitTimer.milliseconds() >= 1400)
                    {
                        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                        elevatorUp = false;
                        drive.turn(Math.toRadians(10));
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                        currentState = State.WAITSHOT1;
                        waitTimer.reset();
                    }
                    break;
                case WAITSHOT1:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAITSHOT2;
                        waitTimer.reset();
                    }
                    break;
                case WAITSHOT2:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAITSHOT3;
                        waitTimer.reset();
                    }
                    break;
                case WAITSHOT3:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        flickerchange = true;
                        flickerTime.reset();
                        currentState = State.WAITFINAL;
                        waitTimer.reset();
                    }
                    break;
                case WAITFINAL:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.followTrajectoryAsync(PARK);
                        currentState = State.IDLE;
                    }
                    break;
                case INTAKE:
                    if (!drive.isBusy()) {
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.TOP);
                        drive.followTrajectoryAsync(shootRing);
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.HIGHGOAL);
                        currentState = State.HIGHSHOT;
                    }
                    break;
                case HIGHSHOT:
                    if (!drive.isBusy()) {
                        //drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                        currentState = State.WAIT6;
                        waitTimer.reset();
                    }
                    break;
                case WAIT6:
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
                        currentState = State.FINISH;
                        waitTimer.reset();

                    }
                    break;
                case FINISH:
                    if (waitTimer.milliseconds() >= 200) {
                        drive.turn(Math.toRadians(-90));
                        //drive.followTrajectoryAsync(parkb);
                        currentState = State.PLACEWOBBLE;
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
                    }
                    break;
                case PLACEWOBBLE:
                    if (!drive.isBusy()) {
                        if (numRings == UGCV.numRings.ONE) {
                            drive.followTrajectoryAsync(strafeToDrop);
                        }
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
                        currentState = State.ENDWOBBLE;
                        waitTimer.reset();
                    }
                    break;
                case ENDWOBBLE:
                    if(waitTimer.milliseconds() >= 400){
                        drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.INIT);
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    drive.robot.getGripperSubsystem().getStateMachine().updateState(GripperStateMachine.State.INIT);
                    drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
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
            } //end for switch

            drive.update();
            drive.getExpansionHubs().update(getDt());
            drive.robot.getShooterSubsystem().update(getDt());
            drive.robot.getFlickerSubsystem().update(getDt());
            drive.robot.getForkliftSubsystem2().update(getDt());
            drive.robot.getGripperSubsystem().update(getDt());
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

    public static TimeProfiler getUpdateRuntime() {return updateRuntime;}
    public static void setUpdateRuntime(TimeProfiler updaRuntime) { updateRuntime = updaRuntime; }
    public static double getDt() { return dt;}
    public static void setDt(double pdt) { dt = pdt; }

}