package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/13/2016.
 */
//@Autonomous(name="Napalm", group="Iterative Opmode")
public class PidLoop extends LinearOpMode {
    //WHY ARE YOU RENAMING THINGS?   PLEASE DON'T BREAK THINGS.
    HardwareMap hw=null;
    //comment
    int currentPhase=0;
    double zCounter=0;
    final double gyroValueWhenStraight=.581622678;
    //For localized control loops.  These values should only be used when needed for a loop.
    double currentRightMotorPower=.2;
    double currentLeftMotorPower=.2;
    double angVel=0;
    //TODO:FIX CONSTANTS AND MAKE ACCURATE.  THESE NEED A LOT OF FINE TUNING
    double Kp=.01; //proportionality constant
    double Kd=.0002; //derivative constant
    double Ki=.001; //integral constant
    double previousAngVel=0;
    double angVelDerivative=0;
    double previousT=-1;
    public void runOpMode() throws InterruptedException{
        HardwareFunctions hf=new HardwareFunctions(hardwareMap);
        waitForStart();

        hf.setColorSensorLight(false);
        while(opModeIsActive()){
            //first phase--go straight until we see the wall
            if (currentPhase==0){
                angVel=hf.getGyroRotation(hf.gyroSensor)-gyroValueWhenStraight;
                angVelDerivative=angVel-previousAngVel;
                previousAngVel=angVel;
                //pid loop
                if(angVel>0){
                    currentLeftMotorPower+=(Kp*angVel)+(Kd*angVelDerivative)+(Ki*zCounter);
                    currentRightMotorPower+=(Kp*angVel)+(Kd*angVelDerivative)+(Ki*zCounter);
                }
                else if(angVel<0){
                    currentLeftMotorPower+=(Kp*angVel)+(Kd*angVelDerivative)+(Ki*zCounter);
                    currentRightMotorPower+=(Kp*angVel)+(Kd*angVelDerivative)+(Ki*zCounter);
                }
                /*if(hf.getUltrasonicData(hf.ultrasonicSensor2)<8){
                hf.runDriveTrain(.15f,.1f);}
                else if(hf.getUltrasonicData(hf.ultrasonicSensor2)>8){
                    hf.runDriveTrain(.1f,.15f);}
                else{
                    hf.runDriveTrain(.2f,.2f);
                }*/
                hf.runDriveTrain((float)currentRightMotorPower,(float)currentLeftMotorPower);
                if(hf.getUltrasonicData(hf.ultrasonicSensor)<16&&hf.getUltrasonicData(hf.ultrasonicSensor)!=0){
                    currentPhase=1;
                    hf.gyroSensor.calibrate();//reset gyro sensor so we know distance to turn
                    hf.gyroSensor.resetZAxisIntegrator();
                }
            }
            //phase 2--turn 90 degrees
            else if(currentPhase==1){
                hf.runDriveTrain(-.5f,0);
                if(previousT==-1){
                    previousT=System.currentTimeMillis();
                }
                //TO DO TEST
                zCounter+=(hf.getGyroRotation(hf.gyroSensor)-gyroValueWhenStraight)*previousT;
                previousT=System.currentTimeMillis()-previousT;
                //800 was chosen based on experimentation.  Choose values to better match
                if(zCounter>800){
                    zCounter=0;
                    currentRightMotorPower=.2;
                    currentLeftMotorPower=.2;
                    currentPhase=2;
                }
            }
            //phase 3--go straight until we see the color sensor
            else if(currentPhase==2){
                //TODO:  MAKE SURE THAT THIS PART OF THE CODE IS WHAT WE WANT.  I SUSPECT THAT IT IS NOT.
                //p(actually technically c loop) loop--update to pid loop maybe?
                if(hf.getGyroRotation(hf.gyroSensor)-gyroValueWhenStraight>0){
                    currentLeftMotorPower-=.001;
                    currentRightMotorPower+=.001;
                }
                else if(hf.getGyroRotation(hf.gyroSensor)-gyroValueWhenStraight<0){//unnecessary conditional
                    currentLeftMotorPower+= .001;
                    currentRightMotorPower-= .001;
                }
                /*if(hf.getUltrasonicData(hf.ultrasonicSensor2)<8){
                hf.runDriveTrain(.15f,.1f);}
                else if(hf.getUltrasonicData(hf.ultrasonicSensor2)>8){
                    hf.runDriveTrain(.1f,.15f);}
                else{
                    hf.runDriveTrain(.2f,.2f);
                }*/
                hf.runDriveTrain((float)currentRightMotorPower,(float)currentLeftMotorPower);
                //TODO: FIX COLOR SENSOR.  COMPLETELY REWRITE AND DESIGN DETECTION OF BEACON,COLORS, and PRESSING OF BUTTON
                if(hf.getColorSensorRed() > 4 && hf.getColorSensorBlue() < hf.getColorSensorRed()){
                    currentPhase=3;
                }
            }
            else if(currentPhase==3){
                hf.runDriveTrain(0,0);
                //press button
            }
            telemetry.addData("currentPhase",currentPhase);
            telemetry.addData("ODS something", hf.getOpticalDistanceSensorData(hf.ODS));//.021 = distance for color sensing
            telemetry.addData("Gyro Rotation",hf.getGyroRotation(hf.gyroSensor)-gyroValueWhenStraight);
            telemetry.addData("zCounter",zCounter);

            telemetry.addData("R",currentRightMotorPower);
            telemetry.addData("L",currentLeftMotorPower);

            telemetry.addData("Ultrasonic Data",hf.getUltrasonicData(hf.ultrasonicSensor));
            telemetry.update();
           /*if(hf.getColorSensorBlue()>.2){
           hf.runDriveTrain(20,20);}
           else if(hf.getColorSensorRed()>.2){
               hf.runDriveTrain(-20,-20);
           } else {
               hf.runDriveTrain(0,0);
           }*/
            //hf.runDriveTrain(hf.getColorSensorBlue(), hf.getColorSensorGreen());
            idle();
        }
    }


}