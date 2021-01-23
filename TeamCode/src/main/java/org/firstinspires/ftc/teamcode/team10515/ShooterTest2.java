package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.team10515.control.ShooterPhysics;

import static java.lang.System.currentTimeMillis;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getEnhancedGamepad1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.setEnhancedGamepad1;

import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.PulleySubsystem;


@TeleOp(name = "Shooter Test2", group = "Test")

public class ShooterTest2 extends UltimateGoalRobot{
    public double shooterSpeed = 0;
    public double shooterSpeedPole = 0;

    public boolean moveShooter = false;

//    private static EnhancedGamepad enhancedGamepad1;

    public long currentTime = 0;
    public long lastTimeA = 0;
    public long lastTimeY = 0;

    //public ElapsedTime btnPressedA;
    //public ElapsedTime btnPressedY;

    public ShooterPhysics shooterPhysics;

    UGMapTest robot = new UGMapTest();
    @Override
    public void start(){
        telemetry.addData("Started", "Ready for Command");
        telemetry.update();
    }

    @Override
    public void init(){
        /* Initialize the hardware map*/
        robot.init(hardwareMap);
        shooterSpeed = 0.53;
        shooterSpeedPole = 0.45;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Init", "Hello Storm Trooper");    //
        updateTelemetry(telemetry);
        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        //btnPressedA = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //btnPressedY = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        shooterPhysics = new ShooterPhysics();
    }

    @Override
    public void loop() {
        super.loop();

//        super.loop();

//        if(getEnhancedGamepad1().isA()){
//            shooterSpeed = ShooterStateMachine.State.IDLE;
//        }
//        else if(getEnhancedGamepad1().isY()){
//            shooterSpeed = ShooterStateMachine.State.SPEED1;
//        }

//        getShooter().getStateMachine().updateState(shooterSpeed);

        // Trial 1

//        if(gamepad1.a){
//            shooterSpeed = 0.25;
//        }
//        else if(gamepad1.b){
//            shooterSpeed = 0.5;
//        }
//        else if(gamepad1.x){
//            shooterSpeed = 0.75;
//        }
//        else if(gamepad1.y){
//            shooterSpeed = 1;
//        }
//        else if(gamepad1.right_bumper){
//            shooterSpeed = 0;
//        }
        /*
        // Trial 2

        if (getEnhancedGamepad1().isA() && btnPressedA.milliseconds() > 250) {
                telemetry.addLine("a pressed");
                shooterSpeed += 0.1;
                btnPressedA.reset();
        } else if (getEnhancedGamepad1().isY() && btnPressedY.milliseconds() > 250) {
//            if(currentTime-lastTimeY >= 250) {
                telemetry.addLine("y pressed");
                shooterSpeed -= 0.1;
                btnPressedY.reset();
//                lastTimeY = currentTimeMillis();
//            }
        } else if (getEnhancedGamepad1().isB()) {
            shooterSpeed = 0;
        } else if (getEnhancedGamepad1().isX()) {
            shooterSpeed = 0.5;
        }
        */

        // Trial 3

//        if(moveShooter) {
//            shooterSpeed = gamepad1.right_trigger;
//        }
//        else{
//            shooterSpeed = 0;
//        }
//
//        if(gamepad1.a){
//            moveShooter = !moveShooter;
//        }

        // Trial 4
        double physicsShooterSpeed = shooterPhysics.getShooterSpeed(1.5748, 0.6604, 1);//62 inches
        double physicsShooterSpeed2 = shooterPhysics.getShooterSpeed(1.8668, 0.6604, 1);//73.5 inches
        double physicsShooterSpeed3 = shooterPhysics.getShooterSpeed(2.2098, 0.6604, 1);//87 inches


        if (getEnhancedGamepad1().isDpad_up()) {
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED1);

        }
        else if (getEnhancedGamepad1().isDpad_right()){
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.SPEED2);

        }
        else if(getEnhancedGamepad1().isDpad_down()){
            getShooterSubsystem().getStateMachine().updateState(ShooterStateMachine.State.IDLE);

        }
//        if(getEnhancedGamepad1().isyJustPressed())
//            shooterSpeed += 0.05;
//        else if (getEnhancedGamepad1().isaJustPressed())
//            shooterSpeed -= 0.5;


        telemetry.addLine("Shooter Physics Speed 1: " + physicsShooterSpeed);
        telemetry.addLine("Shooter Physics Speed 2: " + physicsShooterSpeed2);
        telemetry.addLine("Shooter Physics Speed 3: " + physicsShooterSpeed3);

//        telemetry.addLine("Actual Speed is: " + shooterSpeed);
        telemetry.addLine("Shooter angle: " + shooterPhysics.getShooterAngle(1.524, 0.6604));
        telemetry.update();
        currentTime = currentTimeMillis();
    }

}
