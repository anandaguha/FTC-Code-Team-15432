/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name="Concept: FinalCode", group ="Concept")
@Disabled
public class FinalCode extends LinearOpMode {

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = " AWlS9/z/////AAABmZYM1T0Mjkv7qjhAix38xFgXDvaBvXaJf4gdeBwhqFmXiTWLW+JleQYEpn4bzqHKk5rzOpzvNFmK7+FnfJYZaI7l+bwaCfpQbBWd0NqDhKe65QsKkR594nF8ZqbwUlhYDjLX97f8elu0ZY8jzqyU3mJ3OntbZ1SWspCn/JzF1TjKZ8J6Hc3jc/eI+fLiLhwC1kWAhyp17D3bEQF02P4yCYrkLZfg2A6Y7XTf8x2sIYPu6h4YScz4mY/zCpuQzU8ppqzEZIY0BbV3YLxw1VL2laRnFgXxSlm3foUtXsPb9GVT4jiA7qLp/xcIg41TPp0icUTBiY4Njx6v3t/NjFfajPcEe4eG22tGMdsPmqxMo1IZ ";



    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");




        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
        // blue left
        if (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            //for (VuforiaTrackable trackable : allTrackables)
            {
                VuforiaTrackables targetsRoverRuckus1 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
                VuforiaTrackable blueRover1 = targetsRoverRuckus1.get(0);
                VuforiaTrackable trackable = blueRover1;

                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // declaring all the Variables
                    ElapsedTime runtime = new ElapsedTime();

                    DcMotor leftDrive;
                    DcMotor rightDrive;

                    ColorSensor colorSensorLeft = null;  // First Hardware Device Object
                    ColorSensor colorSensorRight = null;  // Second Hardware Device Object

                    DigitalChannel digitalTouchLeft;  // Third Hardware Device Object
                    DigitalChannel digitalTouchRight;  // Fourth Hardware Device Object

                // configuring all the vars to actual parts on the robot
                // if trying to set up configs on phone use the names given in the green " " marks

                    colorSensorLeft =  hardwareMap.colorSensor.get("Color_Sensor_Left");
                    colorSensorRight = hardwareMap.colorSensor.get("Color_Sensor_Left");

                    leftDrive = hardwareMap.dcMotor.get("Left_Drive");
                    rightDrive = hardwareMap.dcMotor.get("Right_Drive");

                    rightDrive.setDirection(DcMotor.Direction.REVERSE);

                    digitalTouchLeft = hardwareMap.get(DigitalChannel.class, "sensor_digital_left");
                    digitalTouchRight = hardwareMap.get(DigitalChannel.class, "sensor_digital_right");

                }
            }

            {
                //red right
                VuforiaTrackables targetsRoverRuckus1 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
                VuforiaTrackable redFootPrint1 = targetsRoverRuckus1.get(1);
                VuforiaTrackable trackable = redFootPrint1;

                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // declaring all the Variables
                    ElapsedTime runtime = new ElapsedTime();

                    DcMotor leftDrive;
                    DcMotor rightDrive;

                    ColorSensor colorSensorLeft = null;  // First Hardware Device Object
                    ColorSensor colorSensorRight = null;  // Second Hardware Device Object

                    DigitalChannel digitalTouchLeft;  // Third Hardware Device Object
                    DigitalChannel digitalTouchRight;  // Fourth Hardware Device Object

                // configuring all the vars to actual parts on the robot
                // if trying to set up configs on phone use the names given in the green " " marks

                    colorSensorLeft =  hardwareMap.colorSensor.get("Color_Sensor_Left");
                    colorSensorRight = hardwareMap.colorSensor.get("Color_Sensor_Left");

                    leftDrive = hardwareMap.dcMotor.get("Left_Drive");
                    rightDrive = hardwareMap.dcMotor.get("Right_Drive");

                    rightDrive.setDirection(DcMotor.Direction.REVERSE);

                    digitalTouchLeft = hardwareMap.get(DigitalChannel.class, "sensor_digital_left");
                    digitalTouchRight = hardwareMap.get(DigitalChannel.class, "sensor_digital_right");

                }
            }


            {
                VuforiaTrackables targetsRoverRuckus1 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
                VuforiaTrackable FrontCrater1 = targetsRoverRuckus1.get(2);
                VuforiaTrackable trackable = FrontCrater1;

                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // declaring all the Variables
                    ElapsedTime runtime = new ElapsedTime();

                    DcMotor leftDrive;
                    DcMotor rightDrive;

                    ColorSensor colorSensorLeft = null;  // First Hardware Device Object
                    ColorSensor colorSensorRight = null;  // Second Hardware Device Object

                    DigitalChannel digitalTouchLeft;  // Third Hardware Device Object (Touch Sensor)
                    DigitalChannel digitalTouchRight;  // Fourth Hardware Device Object(Touch Sensor)

                // configuring all the vars to actual parts on the robot
                // if trying to set up configs on phone use the names given in the green " " marks

                    colorSensorLeft =  hardwareMap.colorSensor.get("Color_Sensor_Left");
                    colorSensorRight = hardwareMap.colorSensor.get("Color_Sensor_Left");

                    leftDrive = hardwareMap.dcMotor.get("Left_Drive");
                    rightDrive = hardwareMap.dcMotor.get("Right_Drive");

                    rightDrive.setDirection(DcMotor.Direction.REVERSE);

                    digitalTouchLeft = hardwareMap.get(DigitalChannel.class, "sensor_digital_left");
                    digitalTouchRight = hardwareMap.get(DigitalChannel.class, "sensor_digital_right");
                }
            }

            {
                //blue back
                VuforiaTrackables targetsRoverRuckus1 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
                VuforiaTrackable BackSpace1 = targetsRoverRuckus1.get(3);
                VuforiaTrackable trackable = BackSpace1;

                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        // declaring all the Variables
                        ElapsedTime runtime = new ElapsedTime();

                        DcMotor leftDrive;
                        DcMotor rightDrive;

                        ColorSensor colorSensorLeft = null;  // First Hardware Device Object
                        ColorSensor colorSensorRight = null;  // Second Hardware Device Object

                        DigitalChannel digitalTouchLeft;  // Third Hardware Device Object
                        DigitalChannel digitalTouchRight;  // Fourth Hardware Device Object


                        // configuring all the vars to actual parts on the robot
                        // if trying to set up configs on phone use the names given in the green " " marks

                        colorSensorLeft =  hardwareMap.colorSensor.get("Color_Sensor_Left");
                        colorSensorRight = hardwareMap.colorSensor.get("Color_Sensor_Left");

                        leftDrive = hardwareMap.dcMotor.get("Left_Drive");
                        rightDrive = hardwareMap.dcMotor.get("Right_Drive");

                        rightDrive.setDirection(DcMotor.Direction.REVERSE);

                        digitalTouchLeft = hardwareMap.get(DigitalChannel.class, "sensor_digital_left");
                        digitalTouchRight = hardwareMap.get(DigitalChannel.class, "sensor_digital_right");






                    }
            }


        }
    }
}
