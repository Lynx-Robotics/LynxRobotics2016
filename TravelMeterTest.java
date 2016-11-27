package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

@Autonomous(name="Travel Meter Test", group="Iterative Opmode")
public class TravelMeterTest extends LinearOpMode {
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;

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

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        travelMeters(1.0f,1.0f);
        while(opModeIsActive()){
            telemetry.addData("working ", "true");
            telemetry.update();
        }
    }
}
