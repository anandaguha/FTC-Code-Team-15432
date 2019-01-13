/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * Check out:
 * https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorDigitalTouch.java
 */

@TeleOp(name="TouchSensorDouble", group="Sensor")
//@TeleOp(name = "Sensor: Digital touch", group = "Sensor")
//@Disabled
public class TouchSensorDouble extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    //private TouchSensor Push;
    private DigitalChannel digitalTouchLeft;  // First Hardware Device Object
    private DigitalChannel digitalTouchRight;  // Second Hardware Device Object


    @Override
    public void runOpMode() {
        // get a reference to our digitalTouch object.
        digitalTouchLeft = hardwareMap.get(DigitalChannel.class, "sensor_digital_left");
        digitalTouchRight = hardwareMap.get(DigitalChannel.class, "sensor_digital_right");
        leftDrive = hardwareMap.get(DcMotor.class, "left_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_Drive");

        // set digital channel to input mode.
        digitalTouchLeft.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // NUMBER TWEAKS ARE FOR SURE NEEDED
        while (opModeIsActive()) {
            /**
             * This code give the robot 4 situations and helps the robot alinging and eventually should
             * if the digital channel returns true it's HIGH and the button is unpressed.
             */
            while (digitalTouchLeft.getState() == true && digitalTouchRight.getState() == true) {
                telemetry.addData("Digital Touch Sensors", "Both are not pressed");
                leftDrive.setPower(1.00);
                rightDrive.setPower(1.00);
                telemetry.update();
            }
            while (digitalTouchLeft.getState() == false && digitalTouchRight.getState() == true) {
                telemetry.addData("Digital Touch Sensors", "Left touch is only being pressed");
                leftDrive.setPower(0);
                rightDrive.setPower(1.00);
                telemetry.update();
            }
            while (digitalTouchLeft.getState() == true && digitalTouchRight.getState() == false) {
                telemetry.addData("Digital Touch Sensors", "Right touch is only being pressed");
                leftDrive.setPower(1.00);
                rightDrive.setPower(0);
                telemetry.update();
            }

            while (digitalTouchLeft.getState() == false && digitalTouchRight.getState() == false){
                telemetry.addData(" Digital Touch Sensors", "Both are pressed");
                // for single touch sensor
                double power = 0;
                leftDrive.setPower(power);
                rightDrive.setPower(power);
                telemetry.update();
            }
            sleep (2000);
        }
    }
}
