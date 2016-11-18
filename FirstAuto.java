package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Student on 10/18/2016.
 */
@Autonomous(name="Auto", group="Iterative Opmode")
public class FirstAuto extends LinearOpMode {

    //BEFORE YOU CHANGE THIS CODE, TEST IT.  THEN THINK ABOUT WHETHER OR NOT YOUR CHANGE WILL BREAK THE CODE
    //THEN DON'T CHANGE THE CODE
    public GyroSensor gyroSensor=null;
    HardwareMap hwMap=null;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor forkliftMotor;
    DcMotor sweeperMotor;
    DcMotor shooterMotor;
    Servo leftButton;
    Servo rightButton;
    public ColorSensor colorSensorLeft=null;
    public ColorSensor colorSensorRight=null;

    String MYCOLOR="red";

    @Deprecated
    public void turnRightDegrees(double X) {
        float startVal = gyroSensor.getHeading();
        float goal = (float) (startVal + X);
        if (goal > 360) {
            goal -= 360;
        }
        if (goal < 0) {
            goal += 360;
        }
        while(!(Math.abs(gyroSensor.getHeading()-goal) < 18 || Math.abs(gyroSensor.getHeading()-goal+360) < 18 || Math.abs(gyroSensor.getHeading()-goal-360) < 18)&& opModeIsActive()){
            leftMotor.setPower(-.2);
            rightMotor.setPower(+.2);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("goal: ",goal);
            telemetry.addData("Fast",true);
            telemetry.update();
        }
        while(Math.abs(gyroSensor.getHeading()-goal) > 1  && opModeIsActive()){
            leftMotor.setPower(-.07);
            rightMotor.setPower(+.07);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("Slow",true);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    @Deprecated
    public void turnLeftDegrees(double X){
        //turn left
        float startVal = gyroSensor.getHeading();
        float goal = (float) (startVal - X);
        if (goal > 360) {
            goal -= 360;
        }
        if (goal < 0) {
            goal += 360;
        }
        while(!(Math.abs(gyroSensor.getHeading()-goal) < 18 || Math.abs(gyroSensor.getHeading()-goal+360) < 18 || Math.abs(gyroSensor.getHeading()-goal-360) < 18)&& opModeIsActive()){
            leftMotor.setPower(.2);
            rightMotor.setPower(-.2);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("goal: ",goal);
            telemetry.addData("Fast",true);
            telemetry.update();
        }
        while(Math.abs(gyroSensor.getHeading()-goal) > 1  && opModeIsActive()){
            leftMotor.setPower(.07);
            rightMotor.setPower(-.07);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("Slow",true);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //Used
    public void turnToAbsDegree(double degree,double speed,String direction){
        double currentHeading=gyroSensor.getHeading();
        if(direction=="right"||direction=="r"&&degree!=0){
            while(currentHeading>degree && currentHeading>3 && opModeIsActive()){
                    //if we start after the degree and we are turning right then do this until we get to degree 0
                    currentHeading=gyroSensor.getHeading();
                    runDriveTrain((float)speed,(float)-speed);
                    telemetry.addData("heading",currentHeading);
                    telemetry.update();
                }
            while(currentHeading<degree && opModeIsActive()) {
                //turn until we hit the degree
                //may need a correction factor to avoid over shooting
                currentHeading = gyroSensor.getHeading();
                runDriveTrain((float) speed, (float) -speed);
                telemetry.addData("heading", currentHeading);
                telemetry.update();
            }
        }
        if(direction=="right"||direction=="r"){
            //degree=0;
            while(currentHeading<358 && currentHeading>3 && opModeIsActive()){
                currentHeading=gyroSensor.getHeading();
                runDriveTrain((float)speed,(float)-speed);
                telemetry.addData("heading",currentHeading);
                telemetry.update();
            }
        }
        if(direction=="left"||direction=="l") {
            while(currentHeading<degree && opModeIsActive()){
                //if we start before the degree and we are turning left then do this until we get to degree 360
                currentHeading=gyroSensor.getHeading();
                runDriveTrain((float)-speed,(float)speed);
                telemetry.addData("heading",currentHeading);
                telemetry.update();
            }
            while(currentHeading>degree && opModeIsActive()) {//+3 avoid
                //turn until we hit the degree
                //may need a correction factor to avoid over shooting
                currentHeading = gyroSensor.getHeading();
                runDriveTrain((float) -speed, (float) speed);
                telemetry.addData("heading", currentHeading);
                telemetry.update();
            }

        }
        runDriveTrain(0,0);
    }


    public void travelMeters(float d) {
        travelMeters(d, 0.3f);
    }
    public void travelMeters(float d, float power) {
        d*=100;
        //convert d to cm
        int correction = -00;
        float distancePerTick=(10000+correction)/270;//ticks/cm -- experimentally determined
        if(d > 0) {
            encoderDriveWithRamp(d*distancePerTick, power);
        } else {
            //encoderDriveWithRamp(-d*distancePerTick, power);
            encoderDriveWithRamp(-d*distancePerTick, power);
        }
    }
    public void encoderDriveWithRamp(float distance, float maxPower){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float power=0;
        float rampTime=2;//seconds
        //p(id) loop, telemetry
        final float KP = 0.006f;
        final float KI = 0.0001f;
        final float KD = 0.0002f;
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
            //Make sure the motor power is between -1.0 and 1.0
            if(leftcorrection>1.0f){
                leftcorrection=1.0f;
            } else if(leftcorrection<-1.0f){
                leftcorrection=-1.0f;
            }
            if(rightcorrection>1.0f){
                rightcorrection=1.0f;
            } else if(rightcorrection<-1.0f){
                rightcorrection=-1.0f;
            }
            leftMotor.setPower(leftcorrection);
            rightMotor.setPower(rightcorrection);
            //leftMotor.setPower(power);
            //rightMotor.setPower(power);
            //runDriveTrain(leftcorrection, rightcorrection);
            telemetry.addData("Correction Factor:", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("left pos", leftMotorPos);
            telemetry.addData("right pos", rightMotorPos);
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
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    public void pressButton(){
        //once colors are detected line up and press the button
        //line up with button
        String color=null;
        if(colorSensorLeft.blue()>=2 && colorSensorLeft.blue()>colorSensorLeft.red()){
            color="blue";
        }
        else if(colorSensorLeft.red()>=2 && colorSensorLeft.red()>colorSensorLeft.blue()){
            color="red";
        }
        while(color!=MYCOLOR && opModeIsActive()){
            telemetry.addData("Current Color",color);
            telemetry.addData("red",colorSensorLeft.red());
            telemetry.addData("blue",colorSensorLeft.blue());
            telemetry.update();
            runDriveTrain(.5f,.5f);

            if(colorSensorLeft.blue()>=2 && colorSensorLeft.blue()>colorSensorLeft.red()){
                color="blue";
            }
           else if(colorSensorLeft.red()>=2 && colorSensorLeft.red()>colorSensorLeft.blue()){
                color="red";
            }}
        //ADD BUTTON PRESSING
        runDriveTrain(0,0);
        sleep(10000);
        }

    public boolean pingRight() {
        long iTime = System.currentTimeMillis();
        colorSensorRight.enableLed(true);
        boolean flag1 = false;
        boolean flag2 = false;
        while (System.currentTimeMillis()-iTime < 1000 && opModeIsActive()) {
            if(colorSensorRight.red() > 4) {
                flag1 = true;
            }
            if(colorSensorRight.blue() > 4) {
                flag2 = true;
            }
            telemetry.addData("pinging", colorSensorRight.red() + " " + colorSensorRight.blue());
            telemetry.update();
        }
        colorSensorRight.enableLed(false);
        return flag1 && flag2;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
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
        gyroSensor=hardwareMap.gyroSensor.get("gyro");
        colorSensorRight=hardwareMap.colorSensor.get("color_right");
        //colorSensorLeft=hardwareMap.colorSensor.get("color_right");

        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating() && opModeIsActive()) {
            telemetry.addData("Calibrating", true);
            telemetry.update();
        }
        //TODO:  FIX pressButton() function.--LATER.  DON'T WORRY.  Needs detection and servo.
        //TODO: FIX COEFFICIENTS IN THE PID LOOP(encoders per tick is generally ok, may need some fine tuning).  Pid loop constants need to be redone though
        telemetry.addData("Finished Calibrating", true);
        telemetry.update();


        //This stuff is decent
        float turningSpeed=.6f;
        travelMeters(.9f, 1f);
        turnToAbsDegree(270,turningSpeed,"left");
        travelMeters(.7f,1);
        turnToAbsDegree(0,turningSpeed,"right");
        //This stuff is not
        pressButton();
        pressButton();
        //This stuff may or may not be decent
        turnToAbsDegree(150,turningSpeed,"right");//turn to point at center
        travelMeters(2,1);//go to the center and park
        //MAYBE INCLUDE SOME FIRING HERE
        while(opModeIsActive()){
            telemetry.update();
            idle();
        }
    }




}
