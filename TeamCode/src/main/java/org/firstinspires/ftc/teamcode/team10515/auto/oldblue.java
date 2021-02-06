package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(name= "Blue Auto", group = "drive")
public class oldblue extends LinearOpMode {
    int x = 1;
    UGBase drive;
    private double kP = (1/14600d);
    private double SPEED2 = 23000d;
    private static double dt;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new UGBase(hardwareMap);
        drive.flicker1.setPosition(1.0d);
        drive.flicker2.setPosition(0.0d);
        Pose2d startPose = new Pose2d(-63, 19, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();
        UGCV.numRings numRings = drive.getRingsUsingImage(false);
        telemetry.addLine("Num Rings: "+ numRings);
        telemetry.update();
        //numRings = UGCV.numRings.FOUR;
        if (isStopRequested()) return;
        //shooterStartPoleShot();
        drive.elevatorServo.setPosition(0.75);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(2, 14),Math.toRadians(5))
                .addDisplacementMarker(10,() -> {
                    drive.shooter1.setPower(0.63d);
                    drive.shooter2.setPower(0.63d);
//                    drive.getShooterSubsystem().getShooterWheel1().setPower(SPEED2 * kP);
//                    drive.getShooterSubsystem().getShooterWheel2().setPower(SPEED2 * kP);
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
        //flickerv2(SPEED2);
        drive.followTrajectory(traj2);
        sleep(200);
        //flickerv2(SPEED2);
        drive.followTrajectory(traj3);
        sleep(200);
        //flickerv2(SPEED2);
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
//
//    }

}
