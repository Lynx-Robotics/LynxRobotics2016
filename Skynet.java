package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/15/2016.
 */
@Autonomous(name="Skynet", group="Iterative Opmode")
public class Skynet extends LinearOpMode {
    HardwareMap hw=null;
    HardwareFunctions hf = null;
    double gyroValueWhenStraight=.581622678;
    float ticks = 1120;//1440?
    float circumference = (float)(3.81 * 3.81 * Math.PI); //1.5 inches -> cm; also, to whoever wrote this, this actually gives the area
    boolean encoderFlag = true;
    public void encoderDrive(float distance, float power){
        hf.resetEncoders();
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //p(id) loop, telemetry
        final float KP = 0.005f;
        int leftMotorPos=hf.leftMotor.getCurrentPosition();
        int rightMotorPos=hf.leftMotor.getCurrentPosition()+500;

        while((leftMotorPos <= distance || rightMotorPos <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            if(leftMotorPos > rightMotorPos){
                float correction = KP * (leftMotorPos - rightMotorPos);
                hf.runDriveTrain((power - correction), (power + correction));
                telemetry.addData("Correction Facrot Right", correction);
            }else if(leftMotorPos < rightMotorPos){
                float correction = KP * (rightMotorPos - leftMotorPos);
                hf.runDriveTrain((power + correction), (power - correction));
                telemetry.addData("Correction Facrot Left", correction);

            }else{
                hf.runDriveTrain(power, power);
            }

            telemetry.addData("left motor", hf.leftMotor.getCurrentPosition());
            telemetry.addData("right motor", hf.rightMotor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        hf.runDriveTrain(0.0f, 0.0f);
        hf.resetEncoders();
    }
    public void encoderDriveMeters(float distance, float power) {
        encoderDrive(distance * ticks / circumference, power);
    }
    public void travelMeters(float d) {
        travelMeters(d, 1.0f);
    }
    public void travelMeters(float d, float power) {
        d=d*100;
        //convert d to cm
        float distancePerTick=10000/218;//ticks/cm -- experimentally determined
        if(d > 0) {
            encoderDrive(d*distancePerTick, power);
        } else {
            encoderDrive(-d*distancePerTick, power);
        }
    }

    public void turnXDegrees(double X) {
        hf.gyroSensor.calibrate();
        while(hf.gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        hf.runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
        while (zCounter - (gyroPerDegree*X)<0){
            hf.runDriveTrain(-.3f, .3f);
            zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        hf.runDriveTrain(0,0);
    }
    public void turnXDegrees(double X,double gyroPerDegree) {
        //overload function
        hf.gyroSensor.calibrate();
        while(hf.gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        hf.runDriveTrain(-.3f, .3f);
        if(X>0){
        while (zCounter - (gyroPerDegree*X)<0){
            hf.runDriveTrain(-.3f, .3f);
            zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }}
        else{
            while (zCounter - (gyroPerDegree*X)>0){
                hf.runDriveTrain(-.3f, .3f);
                zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
                previousT=System.currentTimeMillis();
                telemetry.addData("zCounter",zCounter);
                telemetry.addData("distance requested",gyroPerDegree*X);
                telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
                telemetry.update();
            }
        }
        hf.runDriveTrain(0,0);
    }
    public double encoderToAngle(double enc) { //converts encoder ticks to angle in degrees
        double wheelDiam = 33.8; //cm between wheels
        float cmPerTick=218/10000; //cm per encoder tick
        double factor = cmPerTick * 360 / Math.PI / wheelDiam; //Mathemtically determined, Theta = factor * encoder ticks
        return enc * factor; //multiply current encoder ticks by factor, gives theta in degrees
    }
    public double angleToEncoder(double angle) { //converts encoder ticks to angle in degrees
        double wheelDiam = 33.8; //cm between wheels
        float cmPerTick=218/10000; //cm per encoder tick
        double factor = cmPerTick * 360 / Math.PI / wheelDiam; //Mathemtically determined, Theta = factor * encoder ticks
        return angle / factor; //multiply current encoder ticks by factor, gives theta in degrees
    }

    public void turnByEncoders(boolean direction, int degrees) { //directoin = true for left is the plan later, not implemented yet
        if(direction) {
            hf.rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            hf.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            hf.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hf.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        float power = .3f;
        float distance = (float) angleToEncoder(degrees);
        hf.resetEncoders();
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //p(id) loop, telemetry
        final float KP = 0.005f;
        while(hf.leftMotor.getCurrentPosition() <= distance || hf.rightMotor.getCurrentPosition() <= distance){

            if(hf.leftMotor.getCurrentPosition() > hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.leftMotor.getCurrentPosition() - hf.rightMotor.getCurrentPosition());
                hf.runDriveTrain((power - correction), (power + correction));

            }else if(hf.leftMotor.getCurrentPosition() < hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.rightMotor.getCurrentPosition() - hf.leftMotor.getCurrentPosition());
                hf.runDriveTrain((power + correction), (power - correction));

            }else{
                hf.runDriveTrain(power, power);
            }

            telemetry.addData("degrees traveled", encoderToAngle(hf.leftMotor.getCurrentPosition()));
            telemetry.update();
        }
        hf.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hf.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        hf.runDriveTrain(0.0f, 0.0f);
        hf.resetEncoders();
    }
    public void fire(){
        //TODO: FIRE SHOTS
    }
    public void pressButton(){
        //TODO: pressButton
    }
    public void runAutonomous(){
        double step=0;//use this to control current movement
        while(opModeIsActive()) {
            if (step == 0) {
                fire();//fire ball, fire code still isn't implemented
                fire();
                step++;
            }
            if (step == 1) {
                //Turn to face toward the wall.
                turnXDegrees(45);
                step++;
            }
            if (step == 2) {
                //Go till we are close to wall.  The 2 needs to be calbrated to be correct
                travelMeters(2);
                step++;
            }
            if (step == 3) {
                //turn almost all around, hopefully level with wall.  This may need modification
                turnXDegrees(315);
                step++;
            }
            if (step == 4) {
                //go until red is detected.  This need a lot more precision
                hf.runDriveTrain(.2f, .2f);
                if (hf.getColorSensorRed() > 4 && hf.getColorSensorBlue() < hf.getColorSensorRed()) {
                    step++;
                }
            }
            if (step == 5) {
                //hit first button
                pressButton();
                step++;
            }
            if (step == 6) {
                //after hitting button keep going
                hf.runDriveTrain(.2f, .2f);
                if (hf.getColorSensorRed() > 4 && hf.getColorSensorBlue() < hf.getColorSensorRed()) {
                    step++;
                }
            }
            if (step == 7) {
                //press second button.  This stage needs more complexity and precision
                pressButton();
                step++;
            }
        }
    }
    public void runOpMode() throws InterruptedException{
        hf=new HardwareFunctions(hardwareMap);
        waitForStart();
        //runAutonomous();
        encoderDriveMeters(1f,.4f);
        }

    }
    /*
    public void calibrateEncoder(float dist, float power) {
        travelMeters(dist, power, false);
        if(hf.leftMotor.getCurrentPosition() > hf.rightMotor.getCurrentPosition()) {

        } else if (hf.leftMotor.getCurrentPosition() < hf.leftMotor.getCurrentPosition()) {

        } else {
            //CORRECTLY CALIBRATED
        }
    } */

