package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Wobble Goal Test", group = "Test")
@Disabled
public class WobbleGoalTest extends UltimateGoalRobot{
    public RevMotor forkliftMotor;
    boolean returnFromTop = false;
    int i = 0;

    public ElapsedTime btnPressedB; //Regular button

    public enum ArmState {
        IDLE,
        MOVE
    }

    public ArmState currentState;

    @Override
    public void start() {

    }

    @Override
    public void init() {
        super.init();
        forkliftMotor = getForkliftSubsystem2().getForkliftMotor();
        btnPressedB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        currentState = ArmState.IDLE;
        getForkliftSubsystem2().update(getDt());
    }

    @Override
    public void loop() {
        super.loop();
//
//        if(getEnhancedGamepad1().isB() && btnPressedB.milliseconds() > 250){
//            btnPressedB.reset();
//            currentState = ArmState.MOVE;
//        }
        if(getEnhancedGamepad2().isLeftBumperLast()){
            i++;
        }
       // WobbleGoalV3();

        telemetry.addLine("State: " + getForkliftSubsystem2().getState());
        telemetry.addLine("Target Angle: " + getForkliftSubsystem2().getState().getAngle());
        telemetry.addLine("Current Angle: " + getForkliftSubsystem2().getCurrentAngle());
        telemetry.addLine("Power: " + getForkliftSubsystem2().getState().getPower(getForkliftSubsystem2().getCurrentAngle()));
        telemetry.addLine("Counter for LeftBumper: " + i);
        telemetry.update();
    }

    public void WobbleGoalV3(){
            switch (currentState) {
                case IDLE:
                    //do nothing
                    break;
                case MOVE:
                    currentState = ArmState.IDLE;
                    switch (getForkliftSubsystem2().getState()) {
                        case INIT:
                            getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
                            returnFromTop = false;
                            break;
                        case ALIGN:
                            if (returnFromTop)
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.INIT);
                            else
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.TOP);
                            break;
                        case TOP:
                            getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN);
                            returnFromTop = true;
                            break;
                    }
                break;
            }
        }
    }


