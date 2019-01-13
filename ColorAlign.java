///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
package org.firstinspires.ftc.teamcode;
//
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// * Check out:
// * https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorDigitalTouch.java
// */



@TeleOp(name="ColorAlign", group="Sensor")
//@TeleOp(name = "Sensor: Digital touch", group = "Sensor")
//@Disabled
public class ColorAlign extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;


    private ColorSensor colorSensorLeft = null;  // First Hardware Device Object
    private ColorSensor colorSensorRight = null;  // Second Hardware Device Object


    @Override
    public void runOpMode() {
        // get a reference to our Color Sesnor object.
        colorSensorLeft =  hardwareMap.colorSensor.get("Color_Sensor_Left");
        colorSensorRight = hardwareMap.colorSensor.get("Color_Sensor_Left");
        leftDrive = hardwareMap.dcMotor.get("Left_Drive");
        rightDrive = hardwareMap.dcMotor.get("Right_Drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // set digital channel to input mode.

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // NUMBER TWEAKS ARE FOR SURE NEEDED
        while (opModeIsActive()) {
            /**
             * This code give the robot 4 situations and helps the robot alinging
             * by using color sensors
             * all colorstatment staments:
             * .red returns red color levels
             * .blue returns blue color levles
             * .green returns green color levles
             * .alpha = luminosity values
             * .argb = combined levels of all red green and blue
             * since there are two diffrent lines blue and red, change the .blue to .red for the red lines of vice versa for the blue ones
             * LED settings (only 2: on or off)
             * color_sensor.enableLed(true); ----- turns on LED light
             * color_sensor.enableLed(false); ---- turns off LED light
             */


            /**
             all the varibales in the follwing code that need to be fine tuned
             */

            double BlueValLeft = colorSensorLeft.blue();        // these two are given by reading
            double BlueValRight = colorSensorRight.blue();       // of the sensors not changeable
           // long waitTimeMili = 500;                    // wait time (given in miliseconds)
            double noColorDrive = 0.5;                 // power given when there is no line visible
            double WheelThatNeedsCorrection = 0.1;     // power given to the wheel that needs to be moved forward
            double WheelThatIsOnLine = 0;               // power given to the wheel that is alredy in place
            /*
             int BlueValLeftPlus = BlueValLeft + 10;
             int BlueValLeftMinus = BlueValLeft - 10;
             int BlueValRightPlus = BlueValRight + 10;
             int BlueValRightMinus = BlueValRight - 10;

            if (BlueValLeftMinus<= BlueValRight <= BlueValLeftPlus) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(noColorDrive);
                rightDrive.setPower(noColorDrive);

            }
            else if (BlueValRight > BlueValLeftPlus ) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(WheelThatIsOnLine);
                rightDrive.setPower(WheelThatNeedsCorrection);
            }
            else if (colorSensorLeft.blue() < 50  && colorSensorRight.blue() > 50 ) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(WheelThatNeedsCorrection);
                rightDrive.setPower(WheelThatIsOnLine);
            }

            else {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                // for single touch sensor
                double power = 0;
                leftDrive.setPower(power);
                rightDrive.setPower(power);

            }
             **/

            if (colorSensorLeft.blue() < 50  && colorSensorRight.blue() < 50 ) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(noColorDrive);
                rightDrive.setPower(noColorDrive);

            }
            else if (colorSensorLeft.blue() >= 50  && colorSensorRight.blue() <= 50 ) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(WheelThatIsOnLine);
                rightDrive.setPower(WheelThatNeedsCorrection);
            }
            else if (colorSensorLeft.blue() < 50  && colorSensorRight.blue() > 50 ) {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                leftDrive.setPower(WheelThatNeedsCorrection);
                rightDrive.setPower(WheelThatIsOnLine);
            }

            else {
                telemetry.addData("Digital Touch Sensors", "Blue Value for left Sensor" + BlueValLeft +"," + "Blue Value for right Sensor" + BlueValRight );
                // for single touch sensor
                double power = 0;
                leftDrive.setPower(power);
                rightDrive.setPower(power);

            }
            telemetry.update();
            //sleep ( waitTimeMili);
        }
    }
}
