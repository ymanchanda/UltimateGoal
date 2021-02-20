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
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "Red Auto 1", group = "drive")
public class RedAuto1 extends LinearOpMode {
    UGBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;
    boolean shooterRunning = false;
    boolean elevatorUp = false;
    boolean forkliftUp = false;
    boolean flickerchange = false;
    boolean goDown = false;
    public int lastEncoderTicks;
    public int currentEncoderTicks = 0;
    public static final int topPosition = 430;
    public static final int maxPosition = 450; //max position

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
        ZONEA,
        WAIT4,
        WAIT5
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new UGBase(hardwareMap);
        drive.setPoseEstimate(startPose);

        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.DOWN);
        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 8), Math.toRadians(5))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(8)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(8)
                .build();
        Trajectory zoneA = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(12, -38, Math.toRadians(180)), Math.toRadians(0))
                .build();
        Trajectory zoneB = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                .build();
        Trajectory zoneC = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(60, -36, Math.toRadians(180)), Math.toRadians(0))
                .build();
        Trajectory parkb = drive.trajectoryBuilder(zoneB.end())
                .back(24)
                .build();
        Trajectory parkC = drive.trajectoryBuilder(zoneC.end())
                .forward(42)
                .build();
        Trajectory release = drive.trajectoryBuilder(zoneA.end())
                .back(1)
                .build();
        waitForStart();

        UGCV.numRings numRings = drive.getRingsUsingImage(true);
        telemetry.addLine("Num Rings: " + numRings);
        telemetry.update();

        if (isStopRequested()) return;

        currentState = State.WOBBLE;
        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_UP);
        drive.robot.getForkliftSubsystem2().update(getDt());

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
                        waitTimer.reset();
                    }
                    break;
                case WAIT0:
                    if (waitTimer.milliseconds() >= 3000) {
                        currentState = State.IDLE;
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
                        if (numRings == UGCV.numRings.ZERO) {
                            currentState = State.ZONEA;
                            drive.followTrajectoryAsync(zoneA);

                        } else if (numRings == UGCV.numRings.ONE) {
                            currentState = State.ZONEA;
                            drive.followTrajectoryAsync(zoneB);
                        } else {
                            currentState = State.ZONEA;
                            drive.followTrajectoryAsync(zoneC);
                        }
                    }
                    break;
                case ZONEA:
                    if (!drive.isBusy()) {
                        drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_DOWN);
                        goDown = true;
                        if (elevatorUp) {
                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                            elevatorUp = false;
                        }
                        if (shooterRunning) {
                            drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                            shooterRunning = false;
                        }
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
                        if (numRings == UGCV.numRings.ONE) {
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(parkb);
                        } else if (numRings == UGCV.numRings.FOUR) {
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(parkC);
                        } else {
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(release);
                        }
                    }
                    break;
//                case WAIT5:
//                    if (waitTimer.milliseconds() >= 1000) {
//                        currentState = State.IDLE;
//                        drive.followTrajectoryAsync(zoneA);
//                    }
//                    break;
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
            drive.robot.getForkliftSubsystem2().update(getDt());
            drive.robot.getPulleySubsystem().update(getDt());
            drive.robot.getIntakeMotorSubsystem().update(getDt());
            WobbleGoal();

            telemetry.addLine("Output" + drive.robot.getShooterSubsystem().getOutput());
            telemetry.addLine("Speed" + drive.robot.getShooterSubsystem().getState().getSpeed());
            telemetry.addLine("Velocity" + drive.robot.getShooterSubsystem().getShooterWheel1().getVelocity());
            telemetry.update();
        } //end of while
        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public static TimeProfiler getUpdateRuntime() {return updateRuntime;}
    public static void setUpdateRuntime(TimeProfiler updaRuntime) { updateRuntime = updaRuntime; }
    public static double getDt() { return dt;}
    public static void setDt(double pdt) { dt = pdt; }

    void WobbleGoal()
    {
        //brake 1st time when it reaches align OR when it reaches the top
        if (reachedUpPosition(maxPosition)) {
            drive.robot.getForkliftSubsystem2().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.DOWN);
        }

        if (lastEncoderTicks - currentEncoderTicks > 0 && reachedUpPosition(topPosition)){
            drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_UP);//Counteract weight of wobble goal
            if (currentEncoderTicks < topPosition) {//Check if too low even after holding
                drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_UP);
            }
        }

        if (reachedDownPosition(0)) {//Check if forklift has reached 0 position
            drive.robot.getForkliftSubsystem2().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.DOWN);
        }

        if (!reachedDownPosition(0) && goDown) {
            drive.robot.getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.DOWN);
        }
        lastEncoderTicks = currentEncoderTicks;
        currentEncoderTicks = drive.robot.getForkliftSubsystem2().getForkliftMotor().getCurrentEncoderTicks();

    }

    public boolean reachedUpPosition(double position) {
        if (drive.robot.getForkliftSubsystem2().getForkliftMotor().getCurrentEncoderTicks() < position)
            return false;
        else
            return true;
    }

    public boolean reachedDownPosition(double position) {
        if (drive.robot.getForkliftSubsystem2().getForkliftMotor().getCurrentEncoderTicks() > position)
            return false;
        else
            return true;
    }

}
