package org.firstinspires.ftc.teamcode.team10515.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team10515.Robot;
import org.firstinspires.ftc.teamcode.team10515.UGAutoRobot;
import org.firstinspires.ftc.teamcode.team10515.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.team10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ForkliftSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.IntakeMotorSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.IntakeServoSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.PulleySubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.UGExpansionHubs;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;



import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

//If the import error (where 5 of them turn red) happens, then go to FILE and Invalidate and Restart Caches
/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class UGBase extends MecanumDrive {

    public UGAutoRobot robot = new UGAutoRobot();
    private UGExpansionHubs expansionHubs;
//    private PulleySubsystem elevatorSubsystem;
//    private FlickerSubsystem flickerSubsystem;
//    private ShooterSubsystem shooterMotors;
//    private ForkliftSubsystem forkliftSubsystem;
//    private IntakeMotorSubsystem intakeMotorSubsystem;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(18, 0, 1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(20, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.6438;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public RevMotor shooter1, shooter2, forkliftMotor, intakeMotor;
    public  RevServo elevatorServo, flicker1, flicker2;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    private static final String VUFORIA_KEY =
            "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
    private VuforiaLocalizer vuforia = null;

    public UGBase(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        robot.init(hardwareMap);
        initVuforia(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
    //    imu = hardwareMap.get(BNO055IMU.class, "imu");
      //  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
   //      BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        setExpansionHubs(new UGExpansionHubs(robot,
                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"))
        );

//        robot.setMotors(new RevMotor[] {
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter 1")), false, false, false, true, Motor.GOBILDA_6000_RPM.getENCODER_TICKS_PER_REVOLUTION(), 120, 1d),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter 2")), false, false, false, true),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Intake Motor")), true, false, false, false, Motor.GOBILDA_1150_RPM.getENCODER_TICKS_PER_REVOLUTION(), 50.8, 2d),
//                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Forklift Motor")), true, true, true, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION()),
//        });
//        robot.setServos(new RevServo[] {
//                new RevServo((ExpansionHubServo)(hardwareMap.get("Elevator Servo"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("Flicker 1"))),
//                new RevServo((ExpansionHubServo)(hardwareMap.get("Flicker 2"))),
//
//
//        });

//        setShooterSubsystem(new ShooterSubsystem(robot.getMotors()[0], robot.getMotors()[1]));
//        setPulleySubsystem(new PulleySubsystem(robot.getServos()[0]));
//        setIntakeMotorSubsystem(new IntakeMotorSubsystem(robot.getMotors()[62]));
//        setForkliftSubsystem(new ForkliftSubsystem(robot.getMotors()[3]));
//        setFlickerSubsystem(new FlickerSubsystem(robot.getServos()[1], robot.getServos()[2]));

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "RL");
        rightRear = hardwareMap.get(DcMotorEx.class, "RR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
//        shooter1 = new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter 1")), false, false, false, true, Motor.GOBILDA_6000_RPM.getENCODER_TICKS_PER_REVOLUTION(), 120, 1d);
//        shooter2 = new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter 2")), false, false, false, true);
//        intakeMotor =  new RevMotor((ExpansionHubMotor)(hardwareMap.get("Intake Motor")), true, false, false, false, Motor.GOBILDA_1150_RPM.getENCODER_TICKS_PER_REVOLUTION(), 50.8, 2d);
//        forkliftMotor = new RevMotor((ExpansionHubMotor)(hardwareMap.get("Forklift Motor")), true, true, true, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION());
//        setShooterSubsystem(new ShooterSubsystem(shooter1, shooter2));
//        setIntakeMotorSubsystem(new IntakeMotorSubsystem(intakeMotor));
//        setForkliftSubsystem(new ForkliftSubsystem(forkliftMotor));

//        elevatorServo = new RevServo((ExpansionHubServo)(hardwareMap.get("Elevator Servo")));
//        flicker1 =  new RevServo((ExpansionHubServo)(hardwareMap.get("Flicker 1")));
//        flicker2 =  new RevServo((ExpansionHubServo)(hardwareMap.get("Flicker 2")));
//
//        setPulleySubsystem(new PulleySubsystem((elevatorServo)));
//        setFlickerSubsystem(new FlickerSubsystem(flicker1,flicker2));
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

    }

//    public IntakeMotorSubsystem getIntakeMotorSubsystem() {
//        return intakeMotorSubsystem;
//    }
//
//    public void setIntakeMotorSubsystem(IntakeMotorSubsystem intakeMotorSubsystem){
//        this.intakeMotorSubsystem = intakeMotorSubsystem;
//    }
//
//    public ShooterSubsystem getShooterSubsystem() { return shooterMotors; }
//
//    public void setShooterSubsystem(ShooterSubsystem shooterMotors){ this.shooterMotors = shooterMotors; }
//
//    public PulleySubsystem getPulleySubsystem() {
//        return elevatorSubsystem;
//    }
//
//    public void setPulleySubsystem(PulleySubsystem elevatorSubsystem) {
//        this.elevatorSubsystem = elevatorSubsystem;
//    }
//
//    public FlickerSubsystem getFlickerSubsystem(){
//        return flickerSubsystem;
//    }
//
//    public void setFlickerSubsystem(FlickerSubsystem flickerSubsystem){
//        this.flickerSubsystem = flickerSubsystem;
//    }
//
//    public ForkliftSubsystem getForkliftSubsystem() {
//        return forkliftSubsystem;
//    }
//
//    public void setForkliftSubsystem(ForkliftSubsystem forkliftSubsystem){
//        this.forkliftSubsystem = forkliftSubsystem;
//    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    public UGCV.numRings getRingsUsingImage(boolean red) {

        UGCV ugvuforia = new UGCV(vuforia);
        UGCV.numRings position = UGCV.numRings.ZERO;

        position = ugvuforia.GetPosition(false,red);
        return position;
    }

    public void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public UGExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(UGExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

}
