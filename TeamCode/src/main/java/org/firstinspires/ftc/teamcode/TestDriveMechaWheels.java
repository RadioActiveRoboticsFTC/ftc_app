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
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

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
 */

@TeleOp(name="Do Not Run This", group="Linear Opmode")
public class TestDriveMechaWheels extends LinearOpMode {

    // Declare OpMode members.
    private Gyroscope imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;

    private DigitalChannel digitalTouch;  // Hardware Device Object
    private Servo servo = null;

    //    // Define class members
    Servo leftservo;
    Servo rightservo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        leftDriveFront  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightDriveFront = hardwareMap.get(DcMotor.class, "FrontRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);

        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // get a reference to our digitalTouch object.
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "testTouch");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //AngularVelocity angularVelocity = imu.getAngularVelocity(AngleUnit.DEGREES);
            //telemetry.addData("Ang Velocity:", " %f", angularVelocity.xRotationRate);


            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            /*
            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }
            */

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


            boolean xPressed = gamepad1.x;
            boolean yPressed = gamepad1.y;

            /*
            if (xPressed) {
                servo.setPosition(0.25);
            }
            if (yPressed) {
                servo.setPosition(0.75);
            }
           */

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1, 1) ;
            rightPower   = Range.clip(drive - turn, -1, 1) ;


            double strafeDirection = gamepad1.left_stick_x;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            if (strafeDirection == 0.0) {
                // this is traditional tank motion
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                leftDriveFront.setPower(leftPower);
                rightDriveFront.setPower(rightPower);
            } else {
                //power is equal to the x value of the joystick so you are taing the - of a - or vice versa
                double power = strafeDirection;
                // TODO: this if statement is not necessary!
                if (strafeDirection > 0.0) {
                    // strafe right
                    leftDrive.setPower(-power);
                    rightDrive.setPower(power);

                    leftDriveFront.setPower(power);
                    rightDriveFront.setPower(-power);
                    telemetry.addData("strafing right", "yes");

                }
                if (strafeDirection < 0.0)    {
                    // strafe left
                    leftDrive.setPower(-power);
                    rightDrive.setPower(power);

                    leftDriveFront.setPower(power);
                    rightDriveFront.setPower(-power);
                    telemetry.addData("strafing left", "yes");

                }
            }




            //double power = 0.5;

            // strafe right
            /*
            leftDrive.setPower(-power);
            rightDrive.setPower(power);

            leftDriveFront.setPower(power);
            rightDriveFront.setPower(-power);
            */
            // strafe left
            /*
            leftDrive.setPower(power);
            rightDrive.setPower(-power);

            leftDriveFront.setPower(-power);
            rightDriveFront.setPower(power);
            */

            leftDrive.getCurrentPosition();
            rightDrive.getCurrentPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left/Right Joysticks", "left (%.2f), right (%.2f)", drive, turn);
            telemetry.addData("left joystick x", strafeDirection);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("X Pressed?", "%s", xPressed);
            telemetry.update();
        }
    }
}
