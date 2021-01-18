package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team10515.states.PulleyStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.PulleySubsystem;


@TeleOp(name = "New Elevator", group = "Test")

public class ElevatorTest2 extends UltimateGoalRobot {
//    public double servoPos = 0;


    static final double COUNTS_PER_MOTOR_REV = 134.4;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 10.25 * 2; // Wobble Goal Mover Height
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*Math.PI);

    @Override
    public void start(){
        telemetry.addData("Started", "Ready for Command");
        telemetry.update();
    }

    @Override
    public void init() {
        super.init();
    }
//hi
    @Override
    public void loop() {
        super.loop();
        if(getEnhancedGamepad1().isyJustPressed()) {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.UP);
        }
        else if (getEnhancedGamepad1().isaJustPressed())
        {
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        }

        if(getPulleySubsystem().getState().equals(PulleyStateMachine.State.UP) && UltimateGoalRobot.elevatorSensor.getDistance(DistanceUnit.INCH) > 4.5){
            getPulleySubsystem().getStateMachine().updateState(PulleyStateMachine.State.DOWN);
        }


        telemetry.addLine("Distance" + UltimateGoalRobot.elevatorSensor.getDistance(DistanceUnit.INCH));
        //when up - 0 ring:4.6-4.7 1 ring: 4.1-4.2 2 ring:3.2-3.3 3 ring: 2.2-2.3
        //when down -
        //telemetry.addLine("Current Position: " + robot.forkliftMotor.getCurrentPosition());
        telemetry.update();
    }
}


