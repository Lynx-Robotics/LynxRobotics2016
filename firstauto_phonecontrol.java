package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import android.app.Activity;
import android.content.Context;
import android.hardware.SensorEventListener;
import android.os.Bundle;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
/**
 * Created by Arpad Kovesdy on 10/18/2016.
 * Don't freak out-It's just a test
 */
@Autonomous(name="PHONE CONTROL", group="Iterative Opmode")
public class firstauto_phonecontrol extends LinearOpMode implements SensorEventListener{

    //Hardware declarations
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor forkliftMotor;
    DcMotor sweeperMotor;
    DcMotor shooterMotor;
    Servo leftButton;
    Servo rightButton;

    public ColorSensor colorSensorLeft=null;
    public ColorSensor colorSensorRight=null;

    //Section for phone control of accelerometer:
    HardwareMap hwMap=null;
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private boolean started = false;

    public float deltaX, deltaY, deltaZ = 0.0f;
    public float lastX, lastY, lastZ = 0.0f;

    private void sensor_init(){
        sensorManager = (SensorManager)hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null) {
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            sensorManager.registerListener(this, accelerometer, sensorManager.SENSOR_DELAY_FASTEST);
            telemetry.addData("accelerometer status", "loaded");
            telemetry.update();
        } else{
            telemetry.addData("accelerometer status", "fail to load");
            telemetry.update();
        }

    }

    @Override
    public void onSensorChanged(SensorEvent event){
        if(opModeIsActive()) {
            deltaX = Math.abs(lastX - event.values[0]);
            deltaY = Math.abs(lastY - event.values[1]);
            deltaZ = Math.abs(lastZ - event.values[2]);
            lastX = event.values[0];
            lastY = event.values[1];
            lastZ = event.values[2];
            if(deltaX < 0.05f){
                deltaX = 0.0f;
            }
            if(deltaY < 0.05f){
                deltaY = 0.0f;
            }
            if(deltaZ < 0.05f){
                deltaZ = 0.0f;
            }
            telemetry.addData("X", deltaX);
            telemetry.addData("Y", deltaY);
            telemetry.addData("Z", deltaZ);
            telemetry.addData("accelerometer status", "updating");
            telemetry.update();
        } else if(started){
            program_stop();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy){

    }

    private void program_stop(){
        sensorManager.unregisterListener(this);
        leftMotor.setPower(0.0f);
        rightMotor.setPower(0.0f);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float last_encoder_left = 0;
        float last_encoder_right = 0;
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        forkliftMotor = hwMap.dcMotor.get("forklift");
        sweeperMotor = hwMap.dcMotor.get("sweeper");
        shooterMotor = hwMap.dcMotor.get("shooter");
        leftButton = hwMap.servo.get("left_button");
        rightButton = hwMap.servo.get("right_button");
        resetEncoders();
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        sensor_init();
        waitForStart();
        started = true;
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        while (opModeIsActive()) {
            telemetry.addData("left_encoder_delta", leftMotor.getCurrentPosition() - last_encoder_left);
            telemetry.addData("right_encoder_delta", leftMotor.getCurrentPosition() - last_encoder_left);
            telemetry.update();
            wait(10);
            idle();
        }
    }

    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}