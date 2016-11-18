package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Student on 9/10/2016.
 */

public class HardwareFunctions{
    HardwareMap hwMap           =  null;
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    public ColorSensor colorSensor = null;
    public OpticalDistanceSensor ODS = null;
    public GyroSensor gyroSensor=null;
    public UltrasonicSensor ultrasonicSensor=null;
    public UltrasonicSensor ultrasonicSensor2=null;
    public HardwareFunctions(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        rightMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        colorSensor.enableLed(false);
        ODS = hwMap.opticalDistanceSensor.get("ODS");
        gyroSensor=hwMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();
        ultrasonicSensor=hardwareMap.ultrasonicSensor.get("ultrasonic");
        ultrasonicSensor2=hardwareMap.ultrasonicSensor.get("ultrasonic2");
    }

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    public void travelMeters(float x) {
        if(x>0) {
            travel(x, true);
        } else {
            travel(-x, false);
        }
    }
    private void travel(float x, boolean direction) {

        leftMotor.setPower(4);
        rightMotor.setPower(4);
    }

    public void enableEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void disableEncoders(){
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //TODO:FILL IN FUNCTIONS
    public void goStraightXDistance(int X, int speed){
        final int motorTicksPerRevolution=1440;
        final double PI=3.14;
        final double RADIUS=1.5;
        final double circumference=PI*RADIUS*2;
        final int ticksPerInch=(int)(motorTicksPerRevolution/circumference);

        final int target = X*ticksPerInch;

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);

        while(leftMotor.getCurrentPosition()<leftMotor.getTargetPosition()){//right motor would work as well
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public void turnXDegrees(float X){

    }
    public double getTouchSensorData(TouchSensor touchSensor){
        return touchSensor.getValue();
    }

    public float getColorSensorBlue(){
        return this.colorSensor.blue();
    }
    public float getColorSensorRed(){
        return this.colorSensor.red();
    }
    public float getColorSensorGreen(){
        return this.colorSensor.green();
    }
    public void setColorSensorLight(boolean b) {
        colorSensor.enableLed(b);
    }
    public double getGyroRotation(GyroSensor gs){
        return gs.getRotationFraction();
    }
    public double getUltrasonicData(UltrasonicSensor us){
        return us.getUltrasonicLevel();
    }
    /*public static class ColorAlgorithm {
        public boolean isSensorInRange() {
            return false;
        }
        public BeaconColor getBeaconColor() { //0 for
            return null;
        }
        public int getRange() { //0 for very close, 1 for small range, 2 for far ranges
            return 0;
        }
        public boolean inRange() { //return false for range
            return false;
        }
    }
    public enum BeaconColor {
        RED,
        BLUE;
    }
    public static class Navigation {

    }*/

    public double getOpticalDistanceSensorData(OpticalDistanceSensor ODS) {
        return ODS.getLightDetected();
    }


}

