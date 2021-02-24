package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine2;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Wobble Goal", group = "Test")
public class ForkliftTest extends UltimateGoalRobot{
    public RevMotor forkliftMotor;

    public ElapsedTime btnPressedB; //Regular button
    public ElapsedTime btnPressedX; //Override button (Sets state to DOWN)

    public enum ArmState {
        IDLE,
        PRESS,
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
        btnPressedX = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        currentState = ArmState.IDLE;
    }

    @Override
    public void loop() {
        super.loop();

        if(getEnhancedGamepad1().isB() && btnPressedB.milliseconds() > 250){
            btnPressedB.reset();
            currentState = ArmState.PRESS;
        }

        WobbleGoalV3();

        telemetry.addLine("State: " + getForkliftSubsystem2().getState());
        telemetry.addLine("Target Angle: " + getForkliftSubsystem2().getState().getAngle());
        telemetry.addLine("Current Angle: " + getForkliftSubsystem2().getCurrentAngle());
        telemetry.update();


    }

    public void WobbleGoalV3(){
            switch (currentState) {
                case IDLE:
                    forkliftMotor.setPower(0);
                    break;

                case PRESS:
                   // getForkliftSubsystem2().setCurrentTicks(forkliftMotor.getCurrentEncoderTicks());
                    currentState = ArmState.MOVE;
                        switch (getForkliftSubsystem2().getState()) {
                            case DOWN:
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_UP);
                                break;
                            case ALIGN_UP:
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.UP);
                                break;
                            case UP:
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.ALIGN_DOWN);
                                break;
                            case ALIGN_DOWN:
                                getForkliftSubsystem2().getStateMachine().updateState(ForkliftStateMachine2.State.DOWN);
                                break;
                        }
                    break;

                case MOVE:
                    double power = getForkliftSubsystem2().getPower();
                    telemetry.addLine("Power: " + power);
                    if(power == 0.0){
                        currentState = ArmState.IDLE;
                    }
                    break;
            }
        }
    }


