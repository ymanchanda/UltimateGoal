package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "RedAuto", group = "drive")
public class RedAuto1 extends LinearOpMode {
    int x = 1;
    UGBase drive;
    private static double dt;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new UGBase(hardwareMap);
        Pose2d startPose = new Pose2d(-63, 15, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();
        UGCV.numRings numRings = drive.getRingsUsingImage(true);
        telemetry.addLine("Num Rings: "+ numRings);
        telemetry.update();
//        if (isStopRequested()) return;
//        //shooterStartPoleShot();
//        drive.elevatorServo.setPosition(0.75);
//        Trajectory traj = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(2, 10),Math.toRadians(5))
//                .addDisplacementMarker(10,() -> {
//                    drive.shooter1.setPower(0.95d);
//                    drive.shooter2.setPower(0.95d);
//                })
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
//                .strafeLeft(8)
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .strafeLeft(8)
//                .build();
//        drive.followTrajectory(traj);
//        sleep(200);
//        flicker();
//        drive.followTrajectory(traj2);
//        sleep(200);
//        flicker();
//        drive.followTrajectory(traj3);
//        sleep(200);
//        flicker();
//        if(x==1){
//            Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
//                    .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
//                    .build();
//            drive.followTrajectory(goToBase);
//        }
//        else{
//            if(x==2){
//                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
//                        .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
//                        .build();
//                drive.followTrajectory(goToBase);
//            }
//            else{
//                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
//                        .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
//                        .build();
//                drive.followTrajectory(goToBase);
//            }
//
//
//        }
        //telemetry.addLine("Rings: " + numRings);
        //telemetry.update();
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
    public void flicker(){
        drive.flicker1.setPosition(0.7d);
        drive.flicker2.setPosition(0.3d);
        sleep(250);
        drive.flicker1.setPosition(1.0d);
        drive.flicker2.setPosition(0.0d);
    }

}
