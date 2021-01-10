package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.team10515.control.ShooterPhysics;
import org.firstinspires.ftc.teamcode.team10515.odometery.drive.StandardTrackingWheelLocalizer;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;


@TeleOp(name = "Odometry TeleOp", group = "Test")

public class TeleOpOdometry extends LinearOpMode{
    public void runOpMode() {
        // Insert whatever initialization your own code does

        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        waitForStart();

        while(opModeIsActive()) {
            // Make sure to call myLocalizer.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            myLocalizer.update();

            myLocalizer.setPoseEstimate(PoseStorage.currentPose); // go to the end of the autonomous code and add a line of
                                                                    // code that is:
                                                                    // PoseStorage.currentPose = drive.getPoseEstimate();
                                                                    //check the learn roadrunner website for more

            // Retrieve your pose
            Pose2d myPose = myLocalizer.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();
            // Insert whatever teleop code you're using

        }
    }
}
