package org.firstinspires.ftc.teamcode.team10515;
//hi
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.EndGameExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStoneGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeMotorStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FoundationStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;

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
 *      Flywheel Intake:
 *          Dpad-left              -> Flywheels set to intake
 *          Dpad-right             -> Flywheels set to outtake
 *          Y-button (pressed)     -> Stop flywheels from running
 *          X-button (pressed)     -> Spit out stone with flywheels
 *      //Vision:
 *          //Left bumper (pressed)  -> Auto feed
 *      Right bumper (pressed) -> Toggle end game extension slides to extend
 *  User 2:
 *      Flywheel Intake:
 *          Y-button (pressed)     -> Stop flywheels from running.
 *          X-button (pressed)     -> Spit out stone with flywheels
 *      Feeder:
 *          Dpad-up                -> Extend feeder to stacked height based on counter class controlled
 *                                    by the second user.
 *          Dpad-down              -> Fully retracts feeder.
 *          Dpad-right             -> Adds stone to stack tracker (NOTE: only changes when feeder is retracted)
 *          Dpad-left              -> Removes stone from stack tracker (NOTE: only changes when feeder is retracted)
 *          Left-trigger           -> Resets stack tracker (NOTE: only changes when feeder is retracted)
 *          Left bumper (pressed)  -> Toggles whether the four-bar sticks out of the robot or not
 *          Right bumper (pressed) -> Releases grip on the stone (NOTE: only works when trying to stack)
 *
 * @see UltimateGoalRobot
 */
@TeleOp(name = "Game Tele-Op", group = "Main")
public class GameTeleop extends UltimateGoalRobot {
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

        //Update Stack tracker
        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
            getStackTracker().removeStoneFromStack();
        } else if(gamepad1.left_trigger > 0.05d) {
            getStackTracker().resetStack();
        }

        //Update Elevator
        if(getEnhancedGamepad2().isyJustPressed()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
        } else if(getEnhancedGamepad2().isaJustPressed()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        }

        //Toggle Shooter
        if(getEnhancedGamepad2().isDpad_up()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);
        }

        else if(getEnhancedGamepad2().isDpad_left()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);
        }
        else if(getEnhancedGamepad2().isDpad_down()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        }

        //Toggle Flicker
        if (getEnhancedGamepad2().isbJustPressed()) {
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HIT);
        }
        else if(getEnhancedGamepad2().isxJustPressed()){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HOLD);
        }
        //Raise and Lower Wobble Goal Mover
        if(getEnhancedGamepad1().isyJustPressed()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
        }
        else if(getEnhancedGamepad1().isaJustPressed()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
        }


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
        telemetry.addLine("Button Status Just Pressed: "+ getEnhancedGamepad1().isyJustPressed());
        telemetry.addLine("Null Status: "+ getEnhancedGamepad1().getGamepad());
        telemetry.addLine("Button Status: "+ getEnhancedGamepad1().isY());
        telemetry.addLine("Button Status Last: "+ getEnhancedGamepad1().isyLast());
        telemetry.update();
    }
}
