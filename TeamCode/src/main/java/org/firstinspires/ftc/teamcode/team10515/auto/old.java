//package org.firstinspires.ftc.teamcode.team10515.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
//import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
//import org.firstinspires.ftc.teamcode.team10515.Robot;
//import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;
//
///*
// * This is an example of a more complex path to really test the tuning.
// */
//@Autonomous(name= "RedAuto", group = "drive")
//public class old extends LinearOpMode {
//    int x = 1;
//    UGBase drive;
//    private double kP = (1/14600d);
//    private double SPEED2 = 23000d;
//    private static TimeProfiler updateRuntime;
//    private static double dt;
//    Trajectory traj;
//    Trajectory traj2;
//    Trajectory traj3;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new UGBase(hardwareMap);
//        Pose2d startPose = new Pose2d(-63, 19, Math.toRadians(0));
//        drive.setPoseEstimate(startPose);
//        setUpdateRuntime(new TimeProfiler(false));
//        drive.flicker1.setPosition(1.0d);
//        drive.flicker2.setPosition(0.0d);
//        waitForStart();
//        getUpdateRuntime().start();
//        drive.elevatorServo.setPosition(0.75);
//        //runShooter(SPEED2);
//        traj = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(2, 14),Math.toRadians(5))
//                .addDisplacementMarker(() -> {
//                    flickerHit();
//                })
//                .build();
//        traj2 = drive.trajectoryBuilder(traj.end())
//                .strafeLeft(8)
//                .addDisplacementMarker(1,() ->{
//                    flickerIn();
//                })
//                .addDisplacementMarker(()->{
//                    flickerHit();
//                })
//
//                .build();
//        drive.followTrajectoryAsync(traj);
//        drive.followTrajectoryAsync(traj2);
//        sleep(100);
//        drive.followTrajectory(traj2);
//        sleep(100);
//        public void loop() {
//            drive.update();
//            runShooter(SPEED2); // Update some lift pid in the background or whatever
//        }
//        //flicker();
//        //UGCV.numRings numRings = drive.getRingsUsingImage(true);
//        //telemetry.addLine("Num Rings: "+ numRings);
//        //telemetry.update();
//
////        drive.getShooterSubsystem().getShooterWheel1().setPower(SPEED2 * kP);
////        drive.getShooterSubsystem().getShooterWheel2().setPower(SPEED2 * kP);
//        drive.getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
//        setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));
//        drive.getShooterSubsystem().update(getDt());
//        telemetry.addLine("Shooter Output: " + drive.getShooterSubsystem().getOutput());
//        telemetry.update();
////        sleep(800);
//        sleep(200);
//        //flickerv2(SPEED2);
//        sleep(200);
//        //flickerv2(SPEED2);
////        if (isStopRequested()) return;
////        //shooterStartPoleShot();
////        drive.elevatorServo.setPosition(0.75);
////        Trajectory traj = drive.trajectoryBuilder(startPose)
////                .splineTo(new Vector2d(2, 10),Math.toRadians(5))
////                .addDisplacementMarker(10,() -> {
////                    drive.shooter1.setPower(0.95d);
////                    drive.shooter2.setPower(0.95d);
////                })
////                .build();
////        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
////                .strafeLeft(8)
////                .build();
////        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
////                .strafeLeft(8)
////                .build();
////        drive.followTrajectory(traj);
////        sleep(200);
////        flicker();
////        drive.followTrajectory(traj2);
////        sleep(200);
////        flicker();
////        drive.followTrajectory(traj3);
////        sleep(200);
////        flicker();
////        if(x==1){
////            Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
////                    .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
////                    .build();
////            drive.followTrajectory(goToBase);
////        }
////        else{
////            if(x==2){
////                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
////                        .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
////                        .build();
////                drive.followTrajectory(goToBase);
////            }
////            else{
////                Trajectory goToBase = drive.trajectoryBuilder(traj3.end())
////                        .splineToConstantHeading(new Vector2d(5,50),Math.toRadians(0))
////                        .build();
////                drive.followTrajectory(goToBase);
////            }
////
////
////        }
//        //telemetry.addLine("Rings: " + numRings);
//        //telemetry.update();
////        drive.followTrajectory(
////                drive.trajectoryBuilder(traj.end(), true)
////                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
////                        .build()
////        );
//    }
//    public void flickerHit(){
//        drive.flicker1.setPosition(0.7d);
//        drive.flicker2.setPosition(0.3d);
//
//    }
//    public void flickerIn(){
//        drive.flicker1.setPosition(1.0d);
//        drive.flicker2.setPosition(0.0d);
//
//    }
//
//    public void runShooter(double speed){
//        double output = 0d;
//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        while (timer.milliseconds() < 9000) {
//            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));
//            drive.getShooterSubsystem().update(getDt());
////            if(timer.milliseconds() > 750) {
////                drive.flicker1.setPosition(0.7d);
////                drive.flicker2.setPosition(0.3d);
////            }
//            //telemetry.addLine("Shooter Output: " + output);
//            telemetry.addLine("Shooter Velocity: " + drive.getShooterSubsystem().getShooterWheel1().getVelocity());
//            telemetry.update();
//        }
//
////        drive.flicker1.setPosition(1.0d);
////        drive.flicker2.setPosition(0.0d);
//
//    }
//    public static TimeProfiler getUpdateRuntime() {
//        return updateRuntime;
//    }
//
//    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
//        updateRuntime = updaRuntime;
//    }
//
//    public static double getDt() {
//        return dt;
//    }
//    public static void setDt(double pdt) {
//        dt = pdt;
//    }
//
//}
