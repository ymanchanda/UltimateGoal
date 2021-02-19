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

    public ForkliftStateMachine2.State currentState;

    @Override
    public void start() {

    }

    @Override
    public void init() {
        super.init();
        forkliftMotor = getForkliftSubsystem2().getForkliftMotor();
        btnPressedB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedX = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        super.loop();

        if(getEnhancedGamepad1().isB() && btnPressedB.milliseconds() > 250){
            btnPressedB.reset();
            currentState = ForkliftStateMachine2.State.ALIGN_UP;
        }
        else if(getEnhancedGamepad1().isX() && btnPressedX.milliseconds() > 250){
            btnPressedX.reset();
        }

        telemetry.update();
        getEnhancedGamepad1().update();
        WobbleGoalV3();
        getForkliftSubsystem2().getStateMachine().updateState(currentState);

    }

    public void WobbleGoalV3(){
            switch (currentState) {
                case DOWN:
                    telemetry.addLine("1");
                    currentState = ForkliftStateMachine2.State.ALIGN_UP;
                    break;
                case ALIGN_UP:
                    telemetry.addLine("2");
                    currentState = ForkliftStateMachine2.State.UP;
                    break;
                case UP:
                    telemetry.addLine("3");
                    currentState = ForkliftStateMachine2.State.ALIGN_DOWN;
                    break;
                case ALIGN_DOWN:
                    telemetry.addLine("2");
                    currentState = ForkliftStateMachine2.State.DOWN;
                    break;
            }
        }
    }


