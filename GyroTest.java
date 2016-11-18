/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * This is an example LinearOpMode that shows how to use
 * the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor
 * is attached to a Device Interface Module I2C channel
 * and is configured with a name of "gyro".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to th org.firstinspires.ftc.teamcodee Driver Station OpMode list
*/
//@Autonomous(name="GyroTest", group="Iterative Opmode")
//to clarify, the hardware isn't made to work.  See the function in encoder test for farther reference.
//hf.getGyroRotation(gyro) gives gyro rotation in z in arbitrary units.
//numerical integration FTW.
//Please keep the naming of sensors and motors UNIFORM
public class GyroTest extends LinearOpMode {
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
        HiTechnicNxtGyroSensor gyro;   // Hardware Device Object
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        double angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (HiTechnicNxtGyroSensor)hwMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();
        //gyro.resetZAxisIntegrator();
        while (opModeIsActive())  {
            leftMotor.setPower(20);
            rightMotor.setPower(-20);

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            //heading = gyro.getHeading();
            angleZ  = gyro.getRotationFraction();

            //telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Rotation Fraction", angleZ);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
