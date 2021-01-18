package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

@TeleOp(name = "Elevator Test", group = "Test")

public class ElevatorTest extends OpMode {
//    public double servoPos = 0;

    public ElapsedTime btnPressedA;
    public ElapsedTime btnPressedY;
    public boolean toggle = true;

    UGMapTest robot = new UGMapTest();

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
        /* Initialize the hardware map*/
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Ultimate Goal Robot");    //
        updateTelemetry(telemetry);

        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }
//hi
    @Override
    public void loop() {
        if(getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250)
        {
            robot.elevatorServo.setPosition(0.8);
            btnPressedY.reset();
        }
        else if(getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
            robot.elevatorServo.setPosition(0.0);
            btnPressedA.reset();
        }

        if(robot.elevatorSensor.getDistance(DistanceUnit.INCH)>4.5 && (robot.elevatorServo.getPosition()> 0.6))
        {
            robot.elevatorServo.setPosition(0.0);
        }



        telemetry.addLine("E Power: " + robot.elevatorServo.getPosition());
        telemetry.addLine("Distance" + robot.elevatorSensor.getDistance(DistanceUnit.INCH));
        //when up - 0 ring:4.6-4.7 1 ring: 4.1-4.2 2 ring:3.2-3.3 3 ring: 2.2-2.3
        //when down -
        //telemetry.addLine("Current Position: " + robot.forkliftMotor.getCurrentPosition());
        telemetry.update();
    }
}


