package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "TestAuto", group = "drive")
public class BlueAuto1 extends LinearOpMode {
    int x = 1;
    UGBase drive;
    private static double dt;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new UGBase(hardwareMap);
        Pose2d startPose = new Pose2d(-63, 15, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();
        UGCV.numRings numRings = drive.getRingsUsingImage(false);
        telemetry.addLine("Num Rings: "+ numRings);
        telemetry.update();
        if (isStopRequested()) return;
        //shooterStartPoleShot();
        drive.elevatorServo.setPosition(0.75);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(2, 10),Math.toRadians(5))
                .addDisplacementMarker(10,() -> {
                    drive.shooter1.setPower(0.95d);
                    drive.shooter2.setPower(0.95d);
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .strafeLeft(8)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(8)
                .build();
        drive.followTrajectory(traj);
        sleep(200);
        flicker();
        drive.followTrajectory(traj2);
        sleep(200);
        flicker();
        drive.followTrajectory(traj3);
        sleep(200);
        flicker();
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
                        .splineToLinearHeading(new Pose2d(-50,24,Math.toRadians(180)),Math.toRadians(0.0))
                        .addDisplacementMarker(15,()-> {
                            drive.intakeMotor.setPower(0.9);
                            drive.elevatorServo.setPosition(0.0);
                        })
                        .build();
                Trajectory fwd = drive.trajectoryBuilder(getRings.end())
                        .forward(6)
                        .build();
                drive.followTrajectory(goToBase);
                sleep(200);
                drive.followTrajectory(getRings);
                sleep(200);
                drive.followTrajectory(fwd);
            }
            else{
                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
                        .splineToConstantHeading(new Vector2d(53,50),Math.toRadians(0))
                        .addDisplacementMarker(10,()->{
                            drive.shooter1.setPower(0.0);
                            drive.shooter2.setPower(0.0);
                        })
                        .build();
                Trajectory getRings = drive.trajectoryBuilder(goToBase.end())
                        .splineToLinearHeading(new Pose2d(-50,24,Math.toRadians(180)), Math.toRadians(0.0))
                        .addDisplacementMarker(15,()-> {
                            drive.intakeMotor.setPower(0.9);
                            drive.elevatorServo.setPosition(0.0);
                        })
                        .build();
                Trajectory fwd = drive.trajectoryBuilder(getRings.end())
                        .forward(6)
                        .build();
                drive.followTrajectory(goToBase);
                sleep(200);
                drive.followTrajectory(getRings);
                sleep(200);
                drive.followTrajectory(fwd);
            }

        }
        telemetry.addLine("Rings: " + numRings);
        telemetry.update();
    }
    public void flicker(){
        drive.flicker1.setPosition(0.7d);
        drive.flicker2.setPosition(0.3d);
        sleep(250);
        drive.flicker1.setPosition(1.0d);
        drive.flicker2.setPosition(0.0d);
    }

}
