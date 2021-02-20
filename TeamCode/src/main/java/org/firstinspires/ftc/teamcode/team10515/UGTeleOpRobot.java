package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team10515.auto.UGBase;
import org.firstinspires.ftc.teamcode.team10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ForkliftSubsystem2;
import org.firstinspires.ftc.teamcode.team10515.subsystems.IntakeMotorSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.IntakeServoSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.PulleySubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Back Right Wheel  -> RR
 *         Back Left Wheel   -> RL
 *     Intake
 *         Left Flywheel  -> INL
 *         Right Flywheel -> INR
 *     Outtake
 *         Left Extension  -> LL
 *         Right Extension -> LR
 * Servo naming convention:
 *     Outtake
 *
 *     End game
 *         Extension Blocker -> EXT
 * Misc. sensors naming convention:

 */
public abstract class UGTeleOpRobot extends Robot {
    private TimeProfiler matchRuntime;
    protected UGBase drive;

    @Override
    public void init() {
        super.init();
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.getExpansionHubs().start();
        drive.robot.getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        drive.getExpansionHubs().update(getDt());
        drive.update();
        drive.robot.getIntakeMotorSubsystem().update(getDt());
        drive.robot.getShooterSubsystem().update(getDt());
        drive.robot.getFlickerSubsystem().update(getDt());
        drive.robot.getPulleySubsystem().update(getDt());
        drive.robot.getForkliftSubsystem2().update(getDt());
    }


    @Override
    public void stop() {
        super.stop();
        drive.getExpansionHubs().stop();
        //drive.robot.getDrive().stop();
        drive.robot.getForkliftSubsystem2().stop();
        drive.robot.getPulleySubsystem().stop();
        drive.robot.getFlickerSubsystem().stop();
        drive.robot.getShooterSubsystem().stop();
        drive.robot.getIntakeMotorSubsystem().stop();
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

}
