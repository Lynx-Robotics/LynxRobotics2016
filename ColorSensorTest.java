package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/13/2016.
 */
@Autonomous(name="ColorSensorTest", group="Iterative Opmode")
public class ColorSensorTest extends LinearOpMode {
    HardwareMap hw=null;
    public ColorSensor colorSensorLeft=null;
    public ColorSensor colorSensorRight=null;


    public void runOpMode() throws InterruptedException{
        waitForStart();
        colorSensorRight=hardwareMap.colorSensor.get("color_right");
        colorSensorLeft=hardwareMap.colorSensor.get("color_left");



        int i = 0;
        colorSensorLeft.enableLed(false);
        colorSensorRight.enableLed(true);
        while(opModeIsActive()){
            /*
            //telemetry.addData("", hf.getColorSensorBlue() + " " + hf.getColorSensorGreen() + " " + hf.getColorSensorRed());
            if(colorSensorLeft.blue() > 4 && colorSensorLeft.blue() > colorSensorLeft.red()) {
                telemetry.addData("(Blue) Blue", colorSensorLeft.blue() + " Red: " + hf.getColorSensorRed());
            } else if (hf.getColorSensorRed() > 4 && hf.getColorSensorBlue() < hf.getColorSensorRed()) {
                telemetry.addData("(Red) Blue", hf.getColorSensorBlue() + " Red: " + hf.getColorSensorRed());
            } else {
                telemetry.addData("(No color) Blue", hf.getColorSensorBlue() + " Red: " + hf.getColorSensorRed());
            }

            telemetry.update();
            */
            printColors();
        }
        idle();
    }

    public void printColors(){
        telemetry.addData("blue: ", colorSensorLeft.blue());
        telemetry.addData("red: ", colorSensorLeft.red());
        telemetry.addData("green: ", colorSensorLeft.green());
        telemetry.addData("alpha:", colorSensorLeft.alpha());
        telemetry.update();
    }

}
