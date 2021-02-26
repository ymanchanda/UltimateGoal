package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name= "XV Blue NumRings", group = "drive")
public class XVBlueNumRings extends LinearOpMode {    UGBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new UGBase(hardwareMap);
        waitForStart();

        //red detection
        UGCV.numRings numRings = drive.getRingsUsingImageandBlueCam(false);
        //blue detection
        //UGCV.numRings numRings = drive.getRingsUsingImageandBlueCam(false);

        telemetry.addLine("Num Rings: " + numRings);
        telemetry.update();

        if (isStopRequested()) return;
    }

    public static TimeProfiler getUpdateRuntime() {return updateRuntime;}
    public static void setUpdateRuntime(TimeProfiler updaRuntime) { updateRuntime = updaRuntime; }
    public static double getDt() { return dt;}
    public static void setDt(double pdt) { dt = pdt; }

}