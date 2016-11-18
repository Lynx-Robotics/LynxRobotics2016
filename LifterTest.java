package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/23/2016.
 */
@Autonomous(name="LIFTING TEST", group="Iterative Opmode")

public class LifterTest extends LinearOpMode {
    HardwareMap hw=null;
    DcMotor forklift=null;


    public void RunMotorForTSeconds(DcMotor m, float secondsToRun){
        forklift.setPower(1f);
        double time=System.currentTimeMillis();
        while(System.currentTimeMillis()-time<secondsToRun*1000){
        }

       forklift.setPower(0f);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        forklift = hardwareMap.dcMotor.get("forklift");
        RunMotorForTSeconds(forklift,5);
    }
}
