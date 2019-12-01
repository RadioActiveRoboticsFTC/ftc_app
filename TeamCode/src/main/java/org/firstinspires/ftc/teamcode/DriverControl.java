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

//import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="Working Remote Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        Robot2019              robot   = new Robot2019();
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");


        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double rightposition = 0.0;
        double leftposition = 0.0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


            boolean xPressed = gamepad1.x;
            //boolean yPressed = gamepad1.y;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, left stick x-values to strafe, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1, 1) ;
            rightPower   = Range.clip(drive - turn, -1, 1) ;

            //values for strafing
            double strafeDirection = gamepad1.left_stick_x;

            // claw arms can be open and closed.

            // TBF: what are the values for the trigger?
            // for now, assume down == 1.0, and up is 0.0
            float leftTriggerValue = gamepad1.left_trigger;


            // Send calculated power to wheels
            //if you are not strafing, drive normally
            if (strafeDirection == 0.0) {

                robot.setPower(leftPower, rightPower);

            }
            //otherwise strafe either left or right, at variable power
            else {
                //power is equal to the x value of the joystick so you are taking the - of a - or vice versa

                robot.setStrafePower(strafeDirection);

            }

            // code for linear slider
            double yAxis = gamepad1.right_stick_y;
            boolean aPressed = gamepad1.a;

            //transition moving up state
            if (yAxis < 0){
                robot.sliderMotor.setPower(yAxis);
            }
            //transition to stall state, change power value TBF
            if(aPressed){
                robot.sliderMotor.setPower(-0.5);
            }
            if(yAxis >= 0 && !aPressed){
                robot.sliderMotor.setPower(0);
            }
            telemetry.addData("left trigger value", leftTriggerValue);

            //open and close the claws of the servo independetly
            if (leftTriggerValue < robot.triggerDownL) {
                leftposition = robot.closedPositionL;
            }
            if (leftTriggerValue == robot.triggerDownL) {
                leftposition = robot.openPositionL;
            }


            // if right trigger pressed right servo goes to closed position

            float rightTriggerValue = gamepad1.right_trigger;
            if (rightTriggerValue < robot.triggerDownR){
                rightposition = robot.closedPositionR;}

            if (rightTriggerValue == robot.triggerDownR){
                rightposition = robot.openPositionR;
            }
            // Display the current value
            telemetry.addData(" Left Servo Position", "%5.2f", leftposition);
            telemetry.addData(">", "Press Stop to end test." );

            robot.leftServo.setPosition(leftposition);
            robot.rightServo.setPosition(rightposition);
            //sleep(CYCLE_MS);
            idle();
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left/Right Joysticks", "left (%.2f), right (%.2f)", drive, turn);
            telemetry.addData("left joystick x", strafeDirection);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("X Pressed?", "%s", xPressed);
            telemetry.addData("RF pos", robot.rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
