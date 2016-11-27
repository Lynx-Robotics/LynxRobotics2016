package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/13/2016.
 */
@Autonomous(name="BlinkTest", group="Iterative Opmode")
public class BlinkTest extends LinearOpMode {
    HardwareMap hw=null;
    //comment
    public void runOpMode() throws InterruptedException{
        HardwareFunctions hf=new HardwareFunctions(hardwareMap);
        waitForStart();

        int i = 0;
        hf.setColorSensorLight(false);
        while(opModeIsActive()){

        }
        idle();
    }


}
