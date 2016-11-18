package org.firstinspires.ftc.teamcode;


/**
 * Created by Student on 10/12/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/10/2016.
 */
@TeleOp(name="Fall Carnival", group="Iterative Opmode")
public class FallCarnival extends LinearOpMode {
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    HardwareMap hwMap           =  null;
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        rightMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            leftMotor.setPower(gamepad1.left_stick_y*.7);
            rightMotor.setPower(gamepad1.right_stick_y*.7);
            idle();
        }
    }
}



