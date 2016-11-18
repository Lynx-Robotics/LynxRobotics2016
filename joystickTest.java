package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by Student on 9/20/2016.
 */
@TeleOp(name="Joystick Final")
public class joystickTest extends LinearOpMode {
    HardwareMap hwMap = null;
   // HardwareFunctions hf = null;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor forkliftMotor;
    DcMotor sweeperMotor;
    DcMotor shooterMotor;
    Servo leftButton;
    Servo rightButton;
    boolean toggleForwardSweeper;
    boolean shootingInProgress,leftButtonInProgress,rightButtonInProgress;
    long leftButtonStartTime, rightButtonStartTime;
    DcMotor greenLED;
    DcMotor blueLED;

    final long programStartTime = System.currentTimeMillis();

    public void runOpMode(){
        shootingInProgress = false;
        leftButtonInProgress = false;
        rightButtonInProgress = false;
        toggleForwardSweeper = false;
        //Init function
        //ODS name: balldetect
        //Encoders: left_drive, right_drive, shooter
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        forkliftMotor = hwMap.dcMotor.get("forklift");
        sweeperMotor = hwMap.dcMotor.get("sweeper");
        shooterMotor = hwMap.dcMotor.get("shooter");
       leftButton = hwMap.servo.get("left_button");
         rightButton = hwMap.servo.get("right_button");
        blueLED=hwMap.dcMotor.get("blue");
        greenLED=hwMap.dcMotor.get("green");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        //NOTHING HAS BEEN TESTED.  NOTHING WORKS.  GOOD LUCK stfu sam
        //TODO: EVERYTHING

        //Looping function
        waitForStart();
        while(opModeIsActive()) {
            //Gamepad 1 is the driver controller and Gamepad 2 is the auxiliary
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);
            forkliftMotor.setPower(gamepad2.left_stick_y);

            if(gamepad1.left_bumper){
                greenLED.setPower(1);
                toggleForwardSweeper = false;
                //Left bumper pressed, go in reverse on sweeper
                sweeperMotor.setDirection(DcMotor.Direction.REVERSE);
                sweeperMotor.setPower(1.0f);
            } else if(gamepad1.right_bumper){
                blueLED.setPower(1);
                //Right bumper pressed, go forward on sweeper (toggle)
                //if(toggleForwardSweeper) toggleForwardSweeper = false;
                //else toggleForwardSweeper = true;
                sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
                sweeperMotor.setPower(1.0f);
            } else{
                blueLED.setPower(0);
                blueLED.setPower(0);
                sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
                sweeperMotor.setPower(0.0f);
            }

            //For forward sweeper, set toggle
            /*
            if(toggleForwardSweeper){
                sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
                sweeperMotor.setPower(1.0f);
            }
            */
                //Make sure shooter isn't shooting already
            if(gamepad2.a||gamepad2.x && !shootingInProgress){
                //If A or X pressed, try to shoot the catapult
                shootingInProgress = true;
                shootWithTime();

                sleep(100);
                shootingInProgress = false;
            }

            if(gamepad2.left_bumper) {
                /*if(!leftButtonInProgress) leftButtonStartTime = System.currentTimeMillis();
                leftButtonInProgress = true;*/
                leftButton.setPosition(0);
            } else {
                leftButton.setPosition(1);
            }

            //Left bumper on Gamepad 2 attempts to extend left button pusher, else retracts it
            if(gamepad2.right_bumper) {
                rightButton.setPosition(0.8);
            } else {
                rightButton.setPosition(0);
            }
        }
    }

    //Function that shoots the catapult;
    public void shoot(){
        resetEncoders();
        double degrees = 25;
        int ticks = (int) Math.ceil(degrees/360*1120); //degree % of revolution multiplied by 1120 ticks, a full revolution
        telemetry.addData("ticks to turn", ticks);
        int startPos = shooterMotor.getCurrentPosition()+45;//currently = 0--+25 to prevent catching

        telemetry.addData("Start Pos", startPos);
        telemetry.addData("Shooter Pos", shooterMotor.getCurrentPosition());
        telemetry.addData("Distance", shooterMotor.getCurrentPosition()-ticks);
        telemetry.addData("Goal", ticks);
        telemetry.update();
        shooterMotor.setPower(1);
        while (shooterMotor.getCurrentPosition()<=ticks && opModeIsActive()) {
            //GET CAUGHT UP UNTIL SHOOTER IS FINISHED
            telemetry.addData("Shooter Pos", shooterMotor.getCurrentPosition());
            telemetry.addData("Distance", shooterMotor.getCurrentPosition()-ticks);
            telemetry.addData("Goal", ticks);
            telemetry.update();
        }
        shooterMotor.setPower(0);
        sleep(10);
        shooterMotor.setPower(-.2f);

        //stops at ~110, does not exit loop
        while (((shooterMotor.getCurrentPosition()>startPos) && opModeIsActive())) {
            //GET CAUGHT UP UNTIL SHOOTER IS FINISHED
            telemetry.addData("Shooter Pos", shooterMotor.getCurrentPosition());
            telemetry.addData("Distance", shooterMotor.getCurrentPosition()-ticks);
            telemetry.addData("Goal", ticks);
            telemetry.update();
        }
        shooterMotor.setPower(0);
    }

    //Alternative function
    public void uniformVelocityShoot(){
        resetEncoders();
        final int startPos = shooterMotor.getCurrentPosition()+45;//a buffer added to stop the catapult early when lowering
        final double c = 0.01; //Constant to change motor power
        final long startTime = System.currentTimeMillis();
        final long timeToShoot = 1000; //Time required to fire completely in ms
        final int encoderTicksToShoot = 600;
        final float shootingRate = encoderTicksToShoot/timeToShoot;
        shooterMotor.setPower(0.8f);

        long timeElapsed = 0;

        while(timeElapsed <= timeToShoot){
            timeElapsed = System.currentTimeMillis() - startTime;

            float target = timeElapsed*shootingRate;
            int actual_distance = shooterMotor.getCurrentPosition();
            double motor_delta = actual_distance - target;

            shooterMotor.setPower( shooterMotor.getPower() - (c*motor_delta) );
            sleep(1);
        }

        //Pull back catapult arm
        sleep(10);

        while (((shooterMotor.getCurrentPosition()>startPos) && opModeIsActive())) {
            shooterMotor.setPower(-.2f);
            sleep(1);
        }
        shooterMotor.setPower(0);
    }
    public void shootWithTime() {
        shooterMotor.setPower(1);
        sleep(130);
        shooterMotor.setPower(0);
        sleep(500);
        shooterMotor.setPower(-1);
        sleep(100);
        shooterMotor.setPower(0);
    }
    //Function that shoots the catapult using the DC Motor encoder
    //This function is not used, do not reference
    @Deprecated
    public void shootWithEncoder(){
        resetEncoders();
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        int shooterMotorPosition = 0;
        while(Math.abs(shooterMotorPosition)<500){
            shooterMotorPosition = shooterMotor.getCurrentPosition();
            shooterMotor.setPower(1);
        }
        shooterMotor.setPower(0);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        while(Math.abs(shooterMotorPosition)<500){
            shooterMotorPosition = shooterMotor.getCurrentPosition();
            shooterMotor.setPower(1);
        }
        shooterMotor.setPower(0);
    }

    public void toggleGreenLEDs(){
    if(greenLED.getPower()==1){
        greenLED.setPower(0);
    }
    if(greenLED.getPower()==0){
        greenLED.setPower(1);
    }
}
    public void toggleBlueLEDs(){
        if(blueLED.getPower()==1){
            blueLED.setPower(0);
        }
        if(blueLED.getPower()==0){
            blueLED.setPower(1);
        }
    }
    //Reset encoder routine for the shooterMotor
    private void resetEncoders(){
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
