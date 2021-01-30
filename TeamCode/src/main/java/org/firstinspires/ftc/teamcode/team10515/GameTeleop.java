package org.firstinspires.ftc.teamcode.team10515;
//hi
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeServoStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;

/**
 * This {@code class} acts as the driver-controlled program for FTC team 10515 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks -> Mecanum drive
 *          Left-trigger           -> Auto foundation movement
 *          Y-button (pressed)     -> Forklift up
 *          X-button (pressed)     -> Forklift down
 *      //Vision:
 *          //Left bumper (pressed)  -> Auto feed
 *      Right bumper (pressed) -> Toggle end game extension slides to extend
 *  User 2:
 *          Dpad-down              -> Stops Shooter
 *          Dpad-right             -> Speed for Power Shots
 *          Dpad-Up                -> Speed for high shots
 *          Right-trigger          -> Turns the Intake On
 *          Left - Trigger         -> Reverses Intake
 *          Back                   -> Stopping the intake
 *          Y - Button             -> Elevator goes up
 *          A - Button             -> Elevator goes down
 *          X - Button             -> Flicker hit rings and come back
 *          Left bumper (pressed)  -> Toggles automation
 *          Right bumper (pressed) -> Intake Servo Hit
 *
 * @see UltimateGoalRobot
 */
@TeleOp(name = "Game Tele-Op", group = "Main")
public class GameTeleop extends UltimateGoalRobot {
    private boolean iselevatorUp = false;
    private boolean isFlicked = false;
    private boolean isAuto = true;
    private boolean doubleCheckUpFlag = false;
    private boolean doubleCheckDownFlag = false;
    private boolean isPushed = false;

    public double upThreshold = 9.0;
    public double downThreshold = 4.5;


    public ElapsedTime btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime btnPressedX = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime btnPressedRightBumper = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime doubleCheckUp = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime doubleCheckDown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void start() {
        //Feeder.setManualControlExtension(() -> gamepad2.b ? 0.5d : gamepad2.x ? -0.5d : 0d);
    }

    @Override
    public void loop() {
        super.loop();
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
        setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, new Rotation2d(gamepad1.right_stick_x, false)));

        //Update flywheel intake
        if(getEnhancedGamepad2().getRight_trigger()>0) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
        } else if(getEnhancedGamepad2().getLeft_trigger() > 0) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.OUTTAKE);
        } else if(gamepad2.back) {
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
        }
        //Intake Servo
        if(getEnhancedGamepad2().isRightBumperLast()){
            getIntakeServoSubsystem().getStateMachine().updateState(IntakeServoStateMachine.State.HIT_RING);
            btnPressedRightBumper.reset();
            isPushed = true;
        }
        if (btnPressedRightBumper.milliseconds()>700 && isPushed){
            getIntakeServoSubsystem().getStateMachine().updateState(IntakeServoStateMachine.State.STANDBY);
        }

        //Update Stack tracker
        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
            getStackTracker().removeStoneFromStack();
        } else if(gamepad1.left_trigger > 0.05d) {
            getStackTracker().resetStack();
        }

        //Update Elevator
        if(getEnhancedGamepad2().isyLast()) {
            iselevatorUp = true;
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
            btnPressedY.reset();

        } else if(getEnhancedGamepad2().isaLast()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
            iselevatorUp = false;
        }

        //Toggle Shooter
        if(getEnhancedGamepad2().isDpad_up()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
        }

        else if(getEnhancedGamepad2().isDpad_right()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
        }
        else if(getEnhancedGamepad2().isDpad_down()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        }

        //Toggle Flicker
        if (getEnhancedGamepad2().isbLast()) {
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        }
        else if(getEnhancedGamepad2().isxLast()){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
            btnPressedX.reset();
            isFlicked = true;
        }
        if (btnPressedX.milliseconds()>400 && isFlicked){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.INIT);
        }

        //Raise and Lower Wobble Goal Mover
        if(getEnhancedGamepad1().isyLast()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
//            getForkliftSubsystem().getForkliftMotor().setTargetPosition(470);
//            getForkliftSubsystem().getForkliftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (getForkliftSubsystem().getForkliftMotor().getPosition()<450 ) {
//                getForkliftSubsystem().getForkliftMotor().setPower(0.3);
//            }
        }
        else if(getEnhancedGamepad1().isaLast()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
        }
        else if(getEnhancedGamepad1().isbLast()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.IDLE);
        }

        if(elevatorSensor.getDistance(DistanceUnit.INCH) > downThreshold && (btnPressedY.milliseconds() > 1250) && (iselevatorUp) && isAuto && !doubleCheckDownFlag){
            doubleCheckDownFlag = true;
            doubleCheckDown.reset();
        }
        if(doubleCheckDownFlag && doubleCheckDown.milliseconds() > 1000) {
            if (elevatorSensor.getDistance(DistanceUnit.INCH) > downThreshold && btnPressedY.milliseconds() > 1250 && iselevatorUp && isAuto) {
                getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
                getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
                getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.INTAKE);
                iselevatorUp = false;
                btnPressedA.reset();
            }
            doubleCheckDownFlag = false;
        }

        if(elevatorSensor.getDistance(DistanceUnit.INCH)< upThreshold && (!iselevatorUp) && (isAuto) && btnPressedA.milliseconds()>1250 && !doubleCheckUpFlag){
            doubleCheckUpFlag = true;
            doubleCheckUp.reset();
        }
        if(doubleCheckUpFlag && doubleCheckUp.milliseconds()>1000){
            if(elevatorSensor.getDistance(DistanceUnit.INCH) < upThreshold && (!iselevatorUp) && (isAuto) && btnPressedA.milliseconds()>1250) {
                getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
                getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
                getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
                iselevatorUp = true;
                btnPressedY.reset();
            }
            doubleCheckUpFlag = false;
        }

//        if(elevatorSensor.getDistance(DistanceUnit.INCH)< 8.8 && (!iselevatorUp) && (isAuto) && btnPressedA.milliseconds()>1250){
//            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
//            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
//            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeMotorStateMachine.State.IDLE);
//            iselevatorUp = true;
//            btnPressedY.reset();
//        }
        if (getEnhancedGamepad2().isLeftBumperLast()){
            isAuto = !isAuto;
        }
//8.7-8.6
        //Check to release grip of stone for stacking
//        if(getEnhancedGamepad2().isRight_bumper() /*&& getStackTracker().getExtensionHeight() == Feeder.getSetpoint()*/) {
//            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.NO_GRIP);
//          //  getFeeder().toggleStoneGripper();
//        }
//
//        if(getEnhancedGamepad2().isStart()){
//            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.GRIP);
//        }
//
//        if(getEnhancedGamepad1().isB()){
//            getFeeder().setDeliveryMode(true);
//        }else if(getEnhancedGamepad1().isX()){
//                getFeeder().setDeliveryMode(false);
//        }
//
//        //Toggle end game extension blocker to extend slides
//        if(getEnhancedGamepad1().isBack()) {
//            getEndGameExtensionSubsystem().getStateMachine().updateState(EndGameExtensionStateMachine.State.RELEASE_SLIDES);
//        }
//
//        if(getEnhancedGamepad2().isDpadDownJustPressed()) {
//            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.GRAB);
//        } else if(getEnhancedGamepad2().isDpadUpJustPressed()) {
//            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.INIT);
//        }
//
//        if(getEnhancedGamepad1().isRightBumperJustPressed()) {
//            Feeder.getCapstoneStateMachine().updateState(CapstoneStateMachine.State.DROP_CAPSTONE);
//        } else if(getEnhancedGamepad1().isLeftBumperJustPressed()) {
//            Feeder.getCapstoneStateMachine().updateState(CapstoneStateMachine.State.DROP_FOUNDATION);
//        } else if(getEnhancedGamepad1().isY()) {
//            Feeder.getCapstoneStateMachine().updateState(CapstoneStateMachine.State.HOLD);
//        }


        //telemetry.addLine("Stones stacked: " + getStackTracker());
        //telemetry.addLine("Stacked Height: " + getStackTracker().getExtensionHeight());
        //telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
        //telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
        //telemetry.addLine("Elevator Position: " + getPulleySubsystem(;
//        telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
//        telemetry.addLine("Left Extension Power: " + getFeeder().getLeftExtension().getLastPower());
//        telemetry.addLine("Right Extension Power: " + getFeeder().getRightExtension().getLastPower());
//        telemetry.addLine("V4B State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
//        telemetry.addLine("Time seen stone: " + getFeeder().getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false));
//        telemetry.addLine("Stone distance: " + getFeeder().getStoneDetector().getDistance(DistanceUnit.INCH));
//        telemetry.addLine("Feeder Extension Constants: " + Feeder.getExtendControlConstants());
//        telemetry.addLine("Extension close to setpoint: " + getFeeder().closeToSetpoint(1 / 4d));
//        telemetry.addLine("Extension Profile: " + (Feeder.getExtensionProfile() != null));
//        if(Feeder.getExtensionProfile() != null) {
//            telemetry.addLine("" + Feeder.getExtensionProfile().getPosition());
//        }
        telemetry.addLine("Elevator Status : "+ iselevatorUp);
        telemetry.addLine("Wobble Goal: " + getForkliftSubsystem().getForkliftMotor().getPosition());
        telemetry.addLine("Elevator Sensor: " + elevatorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addLine("Null Status: "+ getEnhancedGamepad1().getGamepad());
        telemetry.addLine("Button Status: "+ getEnhancedGamepad1().getRight_trigger());
        telemetry.addLine("Button Status Last: "+ getEnhancedGamepad1().isyLast());
        telemetry.addLine("Auto status" + isAuto);
        telemetry.update();
    }
}
