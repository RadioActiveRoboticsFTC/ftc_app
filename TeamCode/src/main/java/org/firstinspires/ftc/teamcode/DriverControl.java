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



@TeleOp(name="Working Remote Control", group="Linear Opmode")
public class DriverControl extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // create our robot object so we have access to motors, etc.
        Robot2019              robot   = new Robot2019();

        // setup the robot motors via the configuration file
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Get the initial value of our slider motor position
        double leftSliderStart = robot.leftSliderMotor.getCurrentPosition();
        double rightSliderStart = robot.rightSliderMotor.getCurrentPosition();

        // initialize the bad slider position flags
        boolean prevRightSliderBad = false;
        boolean prevLeftSliderBad = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // moves the robot
            robotMotion(robot);

            // moves the claw and capstone servo
            powerManipulators(robot);

            // code for linear slider; gather the values needed from the
            // second gamepad
            double yAxis = gamepad2.left_stick_y;
            boolean aPressed =   gamepad2.a;
            boolean rightBumper = gamepad2.right_bumper;

            // for each linear slider motor, move it in the desired direction
            // but also make sure we don't unwind the string
            prevLeftSliderBad = powerSliderMotor(robot.leftSliderMotor,
                    leftSliderStart,
                    prevLeftSliderBad,
                    yAxis,
                    aPressed,
                    rightBumper);

            prevRightSliderBad = powerSliderMotor(robot.rightSliderMotor,
                    rightSliderStart,
                    prevRightSliderBad,
                    yAxis,
                    aPressed,
                    rightBumper);

            idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    // function for moving the robot
    public void robotMotion(Robot2019 robot){

        //grab needed values from gamepad1
        float gear = gamepad1.right_trigger;
        float strafeTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        //create scaling factor for the speed of the robot
        double scale = 1;
        if(gear >= robot.gearTriggerDown) scale = 3.0;

        //variable for the power to each side of the robot
        double leftPower;
        double rightPower;

        //variables for turning and strafing
        double drive;
        double turn;
        double strafePower;

        //now we turn the values from the controller into the
        // robot power through our scale factor
        drive = -leftStickY/scale;
        turn = rightStickX/scale;
        strafePower = leftStickX/scale;

        //This is basic math to decide how the different sides
        // of the robot get power for basic driving
        leftPower    = Range.clip(drive + turn, -1, 1) ;
        rightPower   = Range.clip(drive - turn, -1, 1) ;

        //This decides whether the robot will strafe or
        // drive normally, based of the right trigger (strafeTrigger)
        if(strafeTrigger >= robot.strafeTriggerDown){
            robot.setStrafePower(strafePower);
        }else{
            robot.setPower(leftPower, rightPower);
        }

    }

    // function for controling the claw and capstone servo
    public void powerManipulators(Robot2019 robot){
        //inputs from the gamepad controller
        float leftTriggerValue = gamepad2.left_trigger;
        float rightTriggerValue = gamepad2.right_trigger;
        boolean yButton = gamepad2.y;
        boolean xButton = gamepad2.x;
        //positions of the servos
        double rightposition = 0;
        double leftposition = 0;
        //open and close the claws of the servo independetly
        //left claw
        if (leftTriggerValue < robot.triggerDownL) {
            leftposition = robot.closedPositionL;
        }
        if (leftTriggerValue >= robot.triggerDownL) {
            leftposition = robot.openPositionL;
        }

        // if right trigger pressed right servo goes to closed position
        if (rightTriggerValue < robot.triggerDownR){
            rightposition = robot.closedPositionR;}

        if (rightTriggerValue >= robot.triggerDownR){
            rightposition = robot.openPositionR;
        }
        //If the y button is pressed the servos go
        // to the position to drag the foundation
        if (yButton == true){
            leftposition = robot.straightPositionL;
            rightposition = robot.straitPositionR;
        }
        //If the x button is pressed drop the capstone
        if (xButton) {
            robot.dropCap();
        }else{
            robot.raiseCap();
        }

        robot.leftServo.setPosition(leftposition);
        robot.rightServo.setPosition(rightposition);

    }

    // function for powering a single slider motor
    public boolean powerSliderMotor(DcMotor sliderMotor,
                                    double sliderStart,
                                    boolean previousSliderBad,
                                    double yAxis,
                                    boolean aPressed,
                                    boolean rightBumper) {

        // set the scaling factor for moving slider up
        double scale = 1.0;
        if (rightBumper) scale = 2.0;

        double sliderPos= sliderMotor.getCurrentPosition();

        // telemetry.addData("Slider dist", sliderPos - sliderStart);

        // This is false if the linear slider is within the proper range
        boolean isSliderBad = sliderPos > sliderStart+40;
        if(isSliderBad == false && previousSliderBad == true){
            sliderMotor.setPower(0);
        }
        // save this for the next time around
        previousSliderBad = isSliderBad;

        if(isSliderBad){
            // slider is in a bad position; move it out of it
            sliderMotor.setPower(-0.7);
        }
        else {
            // Here, we translate the driver's intention
            // into motor powers

            //transition moving up state
            if (yAxis < 0) {
                sliderMotor.setPower(yAxis/scale);
            }
            //transition to stall state, change power value TBF
            if (aPressed) {
               sliderMotor.setPower(-0.25);
            }

            if (yAxis >= 0 && !aPressed) {
              sliderMotor.setPower(yAxis / 2);
            }
        }
        return previousSliderBad;
    }
}
