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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.PoseStorage;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "Blue Auto 1", group = "drive")
public class BlueAuto1 extends LinearOpMode {
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
    public static final int topPosition2 = 2020;
    public static final int alignPosition = 1000;
    enum WobbleState {
        ZERO,
        ALIGN,
        TOP
    }

    TestOdo.WobbleState wobbleTo = TestOdo.WobbleState.ZERO;
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
        MIDPOINT,
        WAIT7,
        wobble2,
        WAIT5,
        GETRINGS,
        WAIT8,
        INTAKE,
        HIGHSHOT,
        WAIT6,

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
        Trajectory midpoint = drive.trajectoryBuilder(zoneC.end(),true)
                .splineTo(new Vector2d(-12,19),Math.toRadians(180))
                .build();
        Trajectory wobble2C = drive.trajectoryBuilder(midpoint.end(),true)
                .splineTo(new Vector2d(-60,19),Math.toRadians(170))
                .build();
        Trajectory strafe = drive.trajectoryBuilder(wobble2.end())
                .strafeLeft(18)
                .build();
        Trajectory forward = drive.trajectoryBuilder(strafe.end())
                .forward(40)
                .build();
        Trajectory forwardc = drive.trajectoryBuilder(strafe.end())
                .forward(50)
                .build();
        Trajectory back = drive.trajectoryBuilder(forwardc.end())
                .back(20)
                .build();
        Trajectory shootRing = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(2,48),Math.toRadians(10))
                .build();
        waitForStart();

        UGCV.numRings numRings = UGCV.numRings.FOUR;//drive.getRingsUsingImage(false);
        telemetry.addLine("Num Rings: "+ numRings);
        telemetry.update();

        if (isStopRequested()) return;

        currentState = State.WOBBLE;
        drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTOUP);
        drive.robot.getForkliftSubsystem().update(getDt());

        //currentState = State.TRAJ1;
        //drive.followTrajectoryAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            if (!shooterRunning) {
                drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
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
                        if(numRings == UGCV.numRings.ZERO){
                            currentState = State.GOTOZONE;
                            drive.followTrajectoryAsync(zoneA);

                        }
                        else if(numRings == UGCV.numRings.ONE){
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
                        drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTODOWN);
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
                        if (numRings == UGCV.numRings.ONE) {
                            currentState = State.wobble2;
                            drive.followTrajectoryAsync(wobble2);
                        }
                        else if(numRings == UGCV.numRings.FOUR){
                            currentState = State.MIDPOINT;
                            drive.followTrajectoryAsync(midpoint);
                        }
                        else{
                            currentState = State.IDLE;
                            drive.followTrajectoryAsync(release);
                        }
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
                    if (waitTimer.milliseconds() >= 500) {
                        currentState = State.wobble2;
                        drive.followTrajectoryAsync(wobble2C);
                    }
                    break;
                case wobble2:
                    if (!drive.isBusy()) {
                        drive.robot.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                        drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
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
                        if (numRings == UGCV.numRings.ONE){
                            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                            currentState = State.INTAKE;
                            drive.followTrajectoryAsync(forward);
                        }
                        else{
                            drive.robot.getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                            currentState = State.WAIT8;
                            drive.followTrajectoryAsync(forwardc);
                            waitTimer.reset();
                        }
                    }
                    break;
                case WAIT8:
                    if (waitTimer.milliseconds() >= 200){
                        currentState = State.IDLE;
                        drive.followTrajectoryAsync(back);
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
                    if (waitTimer.milliseconds() >= 500) {
                        currentState = State.IDLE;
                        drive.robot.getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);

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

            telemetry.addLine("Output"+drive.robot.getShooterSubsystem().getOutput());
            telemetry.addLine("Speed"+drive.robot.getShooterSubsystem().getState().getSpeed());
            telemetry.addLine("Velocity"+drive.robot.getShooterSubsystem().getShooterWheel1().getVelocity());
            telemetry.addLine("Elevator up"+elevatorUp);

            telemetry.update();
        } // end of while
        drive.setMotorPowers(0.0,0.0,0.0,0.0);
        PoseStorage.currentPose = drive.getPoseEstimate();
        /*
        if(numRings == UGCV.numRings.ZERO){
            Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
                    .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
                    .addDisplacementMarker(10,()->{
                        drive.shooter1.setPower(0.0);
                        drive.shooter2.setPower(0.0);
                    })
                    .build();
            drive.followTrajectory(goToBase);

        }
        else{
            if(numRings == UGCV.numRings.ONE){
                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
                        .splineToConstantHeading(new Vector2d(29,26),Math.toRadians(0))
                        .addDisplacementMarker(10,()->{
                            drive.shooter1.setPower(0.0);
                            drive.shooter2.setPower(0.0);
                        })
                        .build();
                Trajectory getRings = drive.trajectoryBuilder(goToBase.end())
                        .splineToLinearHeading(new Pose2d(-3,36,Math.toRadians(195)),Math.toRadians(0.0))
                        .addDisplacementMarker(15,()-> {
                            drive.intakeMotor.setPower(0.9);
                            drive.elevatorServo.setPosition(0.0);
                        })
                        .build();
                Trajectory fwd = drive.trajectoryBuilder(getRings.end())
                        .forward(20)
                        .build();
                Trajectory highshoot = drive.trajectoryBuilder(fwd.end())
                        .splineToLinearHeading(new Pose2d(2,36,Math.toRadians(5)),Math.toRadians(0.0))
                        .addDisplacementMarker(15,()-> {
                            drive.intakeMotor.setPower(0.0);
                            drive.elevatorServo.setPosition(0.75);
                            drive.shooter1.setPower(0.7d);
                            drive.shooter2.setPower(0.7d);
                        })
                        .build();
                Trajectory park = drive.trajectoryBuilder(highshoot.end())
                        .forward(6)
                        .build();
                drive.followTrajectory(goToBase);
                sleep(200);
                drive.followTrajectory(getRings);
                sleep(200);
                drive.followTrajectory(fwd);
                sleep(200);
                drive.followTrajectory(highshoot);
                flicker();
                sleep(200);
                drive.followTrajectory(park);

            }
            else{
                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
                        .splineToConstantHeading(new Vector2d(43,50),Math.toRadians(0))
                        .addDisplacementMarker(10,()->{
                            drive.shooter1.setPower(0.0);
                            drive.shooter2.setPower(0.0);
                        })
                        .build();
//                Trajectory getRings = drive.trajectoryBuilder(goToBase.end())
//                        .splineToLinearHeading(new Pose2d(-3,36,Math.toRadians(195)), Math.toRadians(0.0))
//                        .build();
//                Trajectory fwd = drive.trajectoryBuilder(getRings.end())
//                        .forward(25)
//                        .build();
//                Trajectory back = drive.trajectoryBuilder(fwd.end())
//                        .back(5)
//                        .addDisplacementMarker(()-> {
//                            drive.intakeMotor.setPower(0.9);
//                            drive.elevatorServo.setPosition(0.0);
//                        })
//                        .build();
                Trajectory park = drive.trajectoryBuilder(goToBase.end())
                        .back(45)
                        .build();
                drive.followTrajectory(goToBase);
                sleep(200);
                drive.followTrajectory(park);
//                drive.followTrajectory(getRings);
//                sleep(200);
//                drive.followTrajectory(fwd);
//                sleep(200);
//                drive.followTrajectory(back);
//                sleep(200);
//                drive.followTrajectory(intakerings);
            }

        }*/

    }
//    public void flicker(){
//        drive.flicker1.setPosition(0.7d);
//        drive.flicker2.setPosition(0.3d);
//        sleep(250);
//        drive.flicker1.setPosition(1.0d);
//        drive.flicker2.setPosition(0.0d);
//    }
//    public void flickerv2(double speed){
//        double output = 0d;
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//        while (timer.milliseconds() < 500) {
//            //keep motor running at constant speed
//            double error = speed - drive.getShooterSubsystem().getShooterWheel1().getVelocity();
//            output = kP * error;
//            drive.getShooterSubsystem().getShooterWheel1().setPower(output);
//            drive.getShooterSubsystem().getShooterWheel2().setPower(output);
//            if(timer.milliseconds() > 300) {
//                drive.flicker1.setPosition(0.7d);
//                drive.flicker2.setPosition(0.3d);
//            }
//        }
//
//        drive.flicker1.setPosition(1.0d);
//        drive.flicker2.setPosition(0.0d);
//
//    }

    public static TimeProfiler getUpdateRuntime() {return updateRuntime;}
    public static void setUpdateRuntime(TimeProfiler updaRuntime) { updateRuntime = updaRuntime; }
    public static double getDt() { return dt;}
    public static void setDt(double pdt) { dt = pdt; }

    void WobbleGoal()
    {
        //brake 1st time when it reaches align OR when it reaches the top
        if (reachedUpPosition(maxPosition)) {
            drive.robot.getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if (lastEncoderTicks - currentEncoderTicks > 0 && reachedUpPosition(topPosition)){
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.HOLD);//Counteract weight of wobble goal
            if (currentEncoderTicks < topPosition) {//Check if too low even after holding
                drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.AUTOUP);
            }
        }

        if (reachedDownPosition(0)) {//Check if forklift has reached 0 position
            drive.robot.getForkliftSubsystem().getForkliftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if (!reachedDownPosition(0) && goDown) {
            drive.robot.getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
        }
        lastEncoderTicks = currentEncoderTicks;
        currentEncoderTicks = drive.robot.getForkliftSubsystem().getForkliftMotor().getCurrentEncoderTicks();

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
