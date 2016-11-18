package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
//THIS IS THE CLASS THAT SHOULD CONTAIN ALL WORK
//IT IS UNBREAKABLE.  ITS ALL READY PROBABLY BROKEN BEYOND ALL BREAKABLILITY
    @Autonomous(name="TestBase", group="Iterative Opmode")
public class baseClass extends LinearOpMode {
    HardwareMap hwMap           =  null;
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    public ColorSensor colorSensor = null;
    public OpticalDistanceSensor ODS = null;
    public GyroSensor gyroSensor=null;
    public UltrasonicSensor ultrasonicSensor=null;
    public UltrasonicSensor ultrasonicSensor2=null;
    public DcMotor forklift=null;
    void init(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        try{
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        rightMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);}
        catch (Exception e){
            telemetry.addData("ERROR(motor init):",e);
        }
        try{
        colorSensor = hardwareMap.colorSensor.get("color sensor");
        colorSensor.enableLed(false);}
        catch(Exception e){
            telemetry.addData("ERROR(color sensor init):",e);
        }
            try{
                ODS = hwMap.opticalDistanceSensor.get("ODS");}
            catch(Error e){
                telemetry.addData("ERROR(ODS init):",e);
            }
        try{
        gyroSensor=hwMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();}
        catch(Exception e){
            telemetry.addData("ERROR(gyro init):",e);
        }
try{
        ultrasonicSensor=hardwareMap.ultrasonicSensor.get("ultrasonic");}
catch(Exception e){
    telemetry.addData("ERROR(ultrasonic init):",e);
}
    try{
        forklift = hardwareMap.dcMotor.get("forklift");
    }
    catch(Exception e){
            telemetry.addData("ERROR(forklift init):",e);
        }
    }

    /*DRIVE TRAIN FUNCTIONS
    ------------------------------------------------------------------------------------------------------
    ------------------------------------------------------------------------------------------------------
    ------------------------------------------------------------------------------------------------------
    ------------------------------------------------------------------------------------------------------
    ------------------------------------------------------------------------------------------------------
    ------------------------------------------------------------------------------------------------------
    */
    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
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
    /*SENSOR FUNCTIONS
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 */
    public double getTouchSensorData(TouchSensor touchSensor){
        try{
        return touchSensor.getValue();}
        catch(Exception e){
            telemetry.addData("Error(getTouchSensorData): ",e);
        }
        return -1;
    }

    public float getColorSensorBlue(){
        try{
        return this.colorSensor.blue();}
        catch(Exception e){
            telemetry.addData("Error(getColorSensorBlue): ",e);
        }
        return -1;
    }
    public float getColorSensorRed(){
        try{
        return this.colorSensor.red();}
        catch(Exception e){
            telemetry.addData("Error(getColorSensorRed): ",e);
        }
        return -1;
    }
    public float getColorSensorGreen(){
        try{
        return this.colorSensor.green();}
        catch(Exception e){
            telemetry.addData("Error(getColorSensorGreen): ",e);
        }
        return -1;
    }
    public void setColorSensorLight(boolean b) {
        try{
        colorSensor.enableLed(b);}
        catch(Exception e){
            telemetry.addData("Error(setColorSensorLight): ",e);
        }
    }
    public double getGyroRotation(GyroSensor gs){
        try{
        return gs.getRotationFraction();}
        catch(Exception e){
            telemetry.addData("Error(getGyroRotation): ",e);
        }
        return 0;
    }
    public double getUltrasonicData(UltrasonicSensor us){
        try{
        return us.getUltrasonicLevel();}
        catch(Exception e){
            telemetry.addData("Error(getUltrasonicData): ",e);
        }
        return -1;
    }
    public double getOpticalDistanceSensorData(OpticalDistanceSensor ODS) {
        try{
        return ODS.getLightDetected();}
        catch(Exception e){
            telemetry.addData("Error(getODSData): ",e);
        }
        return -1;
    }
    /*SENSOR FUNCTIONS
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 ------------------------------------------------------------------------------------------------------
 */
    public void encoderDrive(float distance, float power){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //p(id) loop, telemetry
        final float KP = 0.005f;
        int leftMotorPos=leftMotor.getCurrentPosition();
        int rightMotorPos=leftMotor.getCurrentPosition()+500;

        while((leftMotorPos <= distance || rightMotorPos <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            if(leftMotorPos > rightMotorPos){
                float correction = KP * (leftMotorPos - rightMotorPos);
                runDriveTrain((power - correction), (power + correction));
                telemetry.addData("Correction Facrot Right", correction);
            }else if(leftMotorPos < rightMotorPos){
                float correction = KP * (rightMotorPos - leftMotorPos);
                runDriveTrain((power + correction), (power - correction));
                telemetry.addData("Correction Facrot Left", correction);

            }else{
                runDriveTrain(power, power);
            }

            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
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
//ASSORTED
    public void RunMotorForTSeconds(DcMotor m, float secondsToRun) {
     try{
        m.setPower(1f);
        double time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < secondsToRun * 1000) {
        }}
         catch(Exception e){
         telemetry.addData("error(runMotorForTSeconds)",e);
         }
     }



//TODO
    public void fire(){
        //TODO: FIRE SHOTS
    }
    public void pressButton(){
        //TODO: pressButton
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        telemetry.update();
        travelMeters(1f);


    }
}

