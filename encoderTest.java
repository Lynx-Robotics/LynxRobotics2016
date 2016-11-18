package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/15/2016.
 */
@Autonomous(name="EncoderTest", group="Iterative Opmode")
public class encoderTest extends LinearOpMode {

    HardwareMap hw=null;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    double gyroValueWhenStraight=.581622678;
    float ticks = 1120;//1440?
    float circumference = (float)(3.81 * 3.81 * Math.PI); //1.5 inches -> cm; also, to whoever wrote this, this actually gives the area
    boolean encoderFlag = true;
    public void encoderDrive(float distance, float power){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //p(id) loop, telemetry
        float KP = 0.005f;
        float KI = 0.001f;
        //starting I--initialize I
        float I = 0.0f;
        long startTime = System.currentTimeMillis();
        while((leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            int leftMotorPos=leftMotor.getCurrentPosition();
            int rightMotorPos=rightMotor.getCurrentPosition();
            long deltaTime = System.currentTimeMillis() - startTime;
            float P = leftMotorPos - rightMotorPos;
            //Calculate integral with change in time from the previous iteration of the loop
            I = -(I + P*deltaTime);
            //Set limits to prevent integral windup
            if(I>100) I=100;
            else if(I<-100) I=-100;
            //Placed directly into the motor functions based on which way it is supposed to respond
            float correction = KP * P + KI * I;
            //Limit motor power from range of 0 to 100
            float leftcorrection = power-correction;
            if(leftcorrection>100) leftcorrection=100; if(leftcorrection<0) leftcorrection=0;
            float rightcorrection = power+correction;
            if(rightcorrection>100) rightcorrection=100; if(rightcorrection<0) rightcorrection=0;
            runDriveTrain(leftcorrection, rightcorrection);
            telemetry.addData("Correction Factor: ", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("KP", P);
            telemetry.addData("I: ", I);
            telemetry.update();
            //Reset timer
            startTime = System.currentTimeMillis();
            //Pause for one millisecond in order to not overwhelm the I loop.
            try{
                Thread.sleep(1);
            } catch(Exception e){

            }
        }
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }




    public void encoderDriveJloop(float distance, float power){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float prevError = 0;
        float counter = 0;
        float correction = .0002f;
        float startTime = System.currentTimeMillis();
        boolean finished = false;
        boolean angle = false;
        boolean finishFlag = false;
        float cCorrection = 0;
        while((leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            int leftMotorPos=leftMotor.getCurrentPosition();
            int rightMotorPos=rightMotor.getCurrentPosition();
            float error = leftMotorPos - rightMotorPos;
            float sign = 1;
            if(!finished && leftMotor.getCurrentPosition() > 100) {
                if(leftMotorPos > rightMotorPos) {
                    sign = 1;
                } else {
                    sign = -1;
                }
            }
            if(!angle) {
                correction = 0.0005f * error;
            }
            correction = correction * sign;

            float deltaError = error - prevError;
            prevError = error;
            if(Math.abs(deltaError) < 20) {
                counter++;
            } else {
                counter = 0;
            }
            if(counter > 1000 && !angle && leftMotor.getCurrentPosition() > 100) {
                angle = true;
                cCorrection = correction; //FIX THIS
            }

            if(error < 5 && !finished && angle && leftMotor.getCurrentPosition() > 100) {
                finished = true;
                correction = cCorrection;
            }
            if(startTime > 100 && angle) {
                correction = correction + .002f * sign;
                startTime = 0;
            }
            //Limit motor power from range of 0 to 100
            float leftcorrection = power-correction;
            float rightcorrection = power+correction;
            runDriveTrain(leftcorrection, rightcorrection);
            //telemetry.addData("Correction Factor: ", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("finished", finished);
            telemetry.addData("correction", correction);
            telemetry.addData("cCorrection", cCorrection);
            //telemetry.addData("KP", P);
            //telemetry.addData("I: ", I);
            telemetry.update();
            //Reset timer
            //startTime = System.currentTimeMillis();
            //Pause for one millisecond in order to not overwhelm the I loop.
            try{
                Thread.sleep(1);
            } catch(Exception e){

            }
        }
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }
    public void encoderDriveMeters(float distance, float power) {
        encoderDrive(distance * ticks / circumference, power);
    }
    public void travelMeters(float d) {
        travelMeters(d, 0.3f);
    }
    public void travelMeters(float d, float power) {
        d=d*100;
        //convert d to cm
        int correction = 0;
        float distancePerTick=(10000+correction)/218;//ticks/cm -- experimentally determined
        if(d > 0) {
            encoderDriveWithRamp(d*distancePerTick, power);
            //encoderDriveJloop(d*distancePerTick, power);
        } else {
            encoderDriveWithRamp(-d*distancePerTick, power);
            //encoderDriveJloop(-d*distancePerTick, power);
        }
    }
    /*
    public void turnXDegreesBasic(double X) {
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
        while (zCounter - (gyroPerDegree*X)<0){
            runDriveTrain(-.3f, .3f);
            zCounter += ((double)getGyroRotation(gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",getGyroRotation(gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        runDriveTrain(0,0);
    }

    public void turnXDegreesuUsingHardCodedValues(double X) {
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
            if(X==90){
                gyroPerDegree=.84;
            }
        if(X==180){
            gyroPerDegree=.9;
        }
        if(X==720){
            gyroPerDegree=.95;
        }
        if(X==45){
            gyroPerDegree=.73;
        }
        if(X==30){
            gyroPerDegree=.67;
        }
        while (zCounter - (gyroPerDegree*X)<0){
            runDriveTrain(-.3f, .3f);
            zCounter += ((double)getGyroRotation(gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",getGyroRotation(gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        runDriveTrain(0,0);
    }
    public void turnXDegrees(double X,double gyroPerDegree) {
        //overload function
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        while (zCounter - (gyroPerDegree*X)<0) {
            runDriveTrain(-.3f, .3f);
            zCounter += ((double) getGyroRotation(gyroSensor) - gyroValueWhenStraight) * (double) (System.currentTimeMillis() - previousT);
            previousT = System.currentTimeMillis();
        }
        runDriveTrain(0,0);
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
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        float power = .3f;
        float distance = (float) angleToEncoder(degrees);
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //p(id) loop, telemetry
        final float KP = 0.005f;
        while(leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance){

            if(leftMotor.getCurrentPosition() > rightMotor.getCurrentPosition()){
                float correction = KP * (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
                runDriveTrain((power - correction), (power + correction));

            }else if(leftMotor.getCurrentPosition() < rightMotor.getCurrentPosition()){
                float correction = KP * (rightMotor.getCurrentPosition() - leftMotor.getCurrentPosition());
                runDriveTrain((power + correction), (power - correction));

            }else{
                runDriveTrain(power, power);
            }

            telemetry.addData("degrees traveled", encoderToAngle(leftMotor.getCurrentPosition()));
            telemetry.update();
        }
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }
    */
    public void runOpMode() throws InterruptedException{
        //hf=new HardwareFunctions(hardwareMap);
        waitForStart();
        //travelMeters(1); //218/10000
        //turnXDegrees(90);
        //calibrateEncoder(1, 0.4f);
       /* for (int i = 0; i < 12*4; i++) {
            turnXDegrees(15,.6f);
            sleep(500);
        }*/
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        travelMeters(1
        );
        while(opModeIsActive()) {
            idle();
        }

    }


    public void encoderDriveWithRamp(float distance, float maxPower){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float power=0;
        float rampTime=2;//seconds
        //p(id) loop, telemetry
        final float KP = 0.0035f;
        final float KI = 0.00009f;
        final float KD = 0.0005f;
        float I = 0.0f;
        long startTime = System.currentTimeMillis();
        float startError = 0;
        while((leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            int leftMotorPos=leftMotor.getCurrentPosition();
            int rightMotorPos=rightMotor.getCurrentPosition();
            long deltaTime = System.currentTimeMillis() - startTime;
            if(power<maxPower){
                power+=maxPower/rampTime*(deltaTime)/100;
            }
            float P = leftMotorPos - rightMotorPos;
            float D = (P - startError)/deltaTime;
            //Calculate integral with change in time from the previous iteration of the loop
            I = -(I + P*deltaTime);
            //Set limits to prevent integral windup
            //if(I>1) I=1;
            //else if(I<-1) I=-1;
            //Placed directly into the motor functions based on which way it is supposed to respond
            float correction = KP * P + KI * I * (power/maxPower) + KD * D;
            //Limit motor power from range of 0 to 100
            float leftcorrection = power-correction;
            if(leftcorrection>100) leftcorrection=100; if(leftcorrection<0) leftcorrection=0;
            float rightcorrection = power+correction;
            if(rightcorrection>100) rightcorrection=100; if(rightcorrection<0) rightcorrection=0;
            leftcorrection*=power/maxPower;
            rightcorrection*=power/maxPower;
            runDriveTrain(leftcorrection, rightcorrection);
            telemetry.addData("Correction Factor: ", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("right correction", rightcorrection);
            telemetry.addData("left correction", rightcorrection);
            telemetry.addData("P", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.update();
            //Reset timer
            startTime = System.currentTimeMillis();
            startError = P;
            //Pause for one millisecond in order to not overwhelm the I loop.
            try{
                Thread.sleep(1);
            } catch(Exception e){

            }
        }
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }

    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    /*
    public void calibrateEncoder(float dist, float power) {
        travelMeters(dist, power, false);
        if(leftMotor.getCurrentPosition() > rightMotor.getCurrentPosition()) {

        } else if (leftMotor.getCurrentPosition() < leftMotor.getCurrentPosition()) {

        } else {
            //CORRECTLY CALIBRATED
        }
    } */
}
