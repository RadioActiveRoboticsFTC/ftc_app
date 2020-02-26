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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// This is the parent class for all autonomouse classes.  It contains all
// the basic functions we need for navigating using encoders and the IMU

@Autonomous(name="Our Drive By Encoder", group="Pushbot")
//@Disabled
public class BaseAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Robot2019              robot   = new Robot2019();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    // Constants used for calculating encoder distance from linear distance
    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 795 ;    // our motor

    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);


    // State used for updating telemetry
    Orientation angles;

    // For this base class, we do nothing
    public void runAutoOpMode() {
        // do a whole lot of nothing.
        // to do something, extend this base class, and overide this function
    }

    @Override
    public void runOpMode() {

        // Here we place all the things we want to happen at the begining
        // of every autonomous run

        // Initialize the drive system variables.
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // get ready to use motor encoders
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftDrive.getCurrentPosition(),
                          robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Call the function that child classes will override in order
        // to do something different for each autonomous mode
        runAutoOpMode();

    }

    // drive straight a certain distance using encoders and IMU
    public void driveStraight(double power, double inches, double timoutSecs) {
        encoderDrive(power, inches, inches, timoutSecs);
    }

    // by default, use proportional correction from gyrosensor to keep robot straight
    public void encoderDrive(double power, double leftInches, double rightInches, double timeoutS) {
        encoderDrive(power, leftInches, rightInches, timeoutS, true);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  This code is based off the example code.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double power,
                             double leftInches, double rightInches,
                             double timeoutS,
                             boolean propStraight) {
        int newLeftTarget;
        int newRightTarget;
        double yAxis;

        // this is the angle we will try to maintain
        double targetYangle = robot.getYAxisAngle();
        double powerCorrection = 0.0;
        double gain = 0.01;
        double angleError = 0.0;

        // debugging
        double avgAngleError = 0.0;
        double avgPowerCorrection = 0.0;
        int numCorrections = 0;

        // we are trying to go straight if the wheels were told to go
        // the same distance
        boolean goingStraight = leftInches == rightInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller.
            // Here we convert from the desired linear distance to travel to motor encoders
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.setTargetPosition(newLeftTarget, newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.setPower(Math.abs(power));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // initialize proportional power correction
                powerCorrection = 0.0;

                // calculate this proportional power correction only if we're
                // going straight and we WANT to correct for angle errors
                if (goingStraight && propStraight) {
                    // what is our error?  How far off are we from our target angle?
                    yAxis = robot.getYAxisAngle();
                    angleError = yAxis - targetYangle;

                    // now correct how we are driving based off this error
                    powerCorrection = angleError * gain;
                    // don't let this get out of control
                    powerCorrection = Range.clip(powerCorrection, -0.5, 0.5) ;

                    // are we going in reverse? if so, swap what we do
                    if (leftInches < 0.0) powerCorrection = -powerCorrection;

                    telemetry.addData("Power Correction", powerCorrection);

                    numCorrections += 1;
                    avgAngleError += Math.abs(angleError);
                    avgPowerCorrection += Math.abs(powerCorrection);

                }

                // actually move the robot now by powering the robot, taking the
                // corrections from the gyro sensor into account
                robot.setPower(power + powerCorrection, power - powerCorrection);

                telemetry.addData("avg angle error", avgAngleError/numCorrections);
                telemetry.addData("avg pwr corr", avgPowerCorrection/numCorrections);

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // This function spins the robot to the left, up to the specified angle using the IMU
    public void spinLeft(double toAngle, double power) {

        // this code turns left
        double yAxisAngle;
        boolean turning = true;
        double prevAngle = robot.getYAxisAngle();

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            // what angle are we at right now
            yAxisAngle = robot.getYAxisAngle();
            if (yAxisAngle < 0 && prevAngle > 0) turning = false;
            prevAngle = yAxisAngle;

            // have we spun far enough yet?
            if (yAxisAngle >= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("turning:", turning);
            telemetry.update();

            if (turning) {
                // spin left
                robot.setPower(-power, power);
            }
        }

        //Brake
        robot.setPower(power,-power);
        sleep(10);
        robot.setPower(0);


    }

    // This function spins the robot to the left, up to the specified angle using the IMU
    // TBF: can we combine this function with spinLeft into one?
    public void spinRight(double toAngle, double power) {

        // this code turns left
        double yAxisAngle;
        boolean turning = true;

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            // what angle are we at right now?
            yAxisAngle = robot.getYAxisAngle();

            if (yAxisAngle <= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("turning:", turning);
            telemetry.update();

            if (turning) {
                // spin right
                robot.setPower(power, -power);
            }
        }

        //Brake
        robot.setPower(power,-power);
        sleep(10);
        robot.setPower(0);
    }

    // This function is just like spinLeft, but sets the power proportional to the distance
    // from our target angle; this way we don't have to brake at the end.
    public void spinLeftP(double toAngle, double power) {

        double yAxisAngle;
        yAxisAngle = robot.getYAxisAngle();

        // how far are we trying to go?
        double totalAngularDistance = toAngle-yAxisAngle;
        boolean turning = true;
        double prevAngle = robot.getYAxisAngle();

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            //what angle are we at right now?
            yAxisAngle = robot.getYAxisAngle();

            // watch for the 180/-180 border!
            // Recall that forward is 0 degrees, with positive to the left,
            // negative to the right
            if (yAxisAngle < 0 && prevAngle > 0) turning = false;

            prevAngle = yAxisAngle;

            // proportional power
            double percentAngularDistance = (toAngle-yAxisAngle)/totalAngularDistance;
            double proportionalPower = percentAngularDistance * power;
            // make sure we never get down to a power of zero, or we
            // might never finish our turn
            proportionalPower = Range.clip(proportionalPower, .05, power);

            // have we spun far enough yet?
            if (yAxisAngle >= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("turning:", turning);
            telemetry.addData("pPower", proportionalPower);
            telemetry.update();

            if (turning) {
                // spin left
                robot.setPower(-proportionalPower, proportionalPower);
            }
        }
        robot.setPower(0);
    }

    // This function is just like spinRight, but sets the power proportional to the distance
    // from our target angle; this way we don't have to brake at the end.
    // TBF: can we combine this with spinLeftP to make one function?
    public void spinRightP (double toAngle, double power) {

        // this code turns right
        double yAxisAngle;
        yAxisAngle = robot.getYAxisAngle();
        boolean turning = true;
        double totalAngularDistance = toAngle-yAxisAngle;

        double prevAngle = robot.getYAxisAngle();

        // Loop and update the dashboard
        while (opModeIsActive() && turning) {
            // what angle are we at right now?
           yAxisAngle = robot.getYAxisAngle();

            // watch for the border at -180/180
            if (yAxisAngle > 0 && prevAngle < 0) turning = false;

            prevAngle = yAxisAngle;

            // proportional power
            double percentAngularDistance = (toAngle-yAxisAngle)/totalAngularDistance;
            double proportionalPower = percentAngularDistance * power;
            // make sure we never go to a power of zero and never finish our turning
            proportionalPower = Range.clip(proportionalPower, .05, power);

            // have we spun enough yet?
            if (yAxisAngle <= toAngle) turning = false;

            telemetry.addData("yAxis", yAxisAngle);
            telemetry.addData("pPower", proportionalPower);
            telemetry.addData("turning:", turning);
            telemetry.update();

            if (turning) {
                // spin right
                robot.setPower(proportionalPower, -proportionalPower);
            }
        }

        robot.setPower(0);
    }

    // this function spins the robot left or right
    void spin(boolean spinRight, double toAngle, double power) {
        if (spinRight) {
            spinRightP(-toAngle, power);
        } else {
            spinLeftP(toAngle, power);
        }
    }

    // this program moves the foundation into the building zone,
    // for both sides depending on 'blueSide' input variable
    void moveFoundation(boolean blueSide)
    {
        double straightSpeed = .5;

        // in theory, we know how far we need to go,
        // but in practice we are off by 'offset'
        double offset = 2.0;
        double robotWidth = 18.0;
        double tileWidth = 24.0;
        double dist = (2*tileWidth) - robotWidth - offset;

        // Place the robot with it's back to the wall, adjacent to the building site.
        // Then, move up to the foundation so that we are positioned
        // to drop our claws on top of it.
        // The purpose of this next group of code is to start
        // in a legal position, but then end in the middle of the foundation.
        driveStraight(straightSpeed,dist/3,20);
        robot.brake(1);

         // spinLeftP(45,0.8);
         spin(blueSide,45,0.8);

        driveStraight(straightSpeed,(dist/3)+5,20);
        robot.brake(1);

        // spinRightP(0,0.8);
        spin(!blueSide,0,0.8);

        driveStraight(straightSpeed,dist/3 ,20);
        robot.brake(1);

        // close claws so that they will drop down on top of the foundation
        robot.straightClaws();

        sleep(500);

        // go backwards so that the grooves catch on the foundation edge,
        // and we can start dragging the foundation towards the building site.
        //  driveStraight(0.2,-(dist - 4),20 );
        // slow down a little bit to make sure we grab the foundation
        driveStraight(0.1,-4,20 );
        driveStraight(0.2,-(dist - 4 - 4),20 );

        // this part of the code unattaches the robot from the
        // foundation by lifting linear slider, and opening claws,
        // and letting go of slider
        robot.setLinearMotorPower(-1.0);
        sleep(1000);
        robot.openClaws();

        //drop linear slider
        robot.setLinearMotorPower(.2);

        // now we need to get to the other side of the side of the foundation,
        // so we can push it further into the building zone
        spin(!blueSide, 90, .8);

        // stop dropping the slider
        robot.setLinearMotorPower(0);

        driveStraight(.5,28,10);
        robot.brake(1);

        // spinLeftP(0,.8);
        spin(blueSide, 0, .8);

        driveStraight(.75, 44,20);
        robot.brake(1);

        // spinLeftP(90, .8);
        spin(blueSide, 90, .8);

        // Go far enough to get behind the foundation so
        // that we can push it at the right spot.
        // 12 is too close to the middle of foundation?
        // 6 barely gets the side
        driveStraight(.75,9, 20);
        robot.brake(1);

        // spinLeftP(180, .8);
        spin(blueSide, 180, .8);

        // push the foundation into the building zone
        // is too high a power causing it to skid?
        // It is too risky to use the gyrosensor, when we are at the 180/-180 angle boundary.
        // TBF: reset the gyro sensor here, rather then simply not use it.
        //  driveStraight(.2,50, 20);
        encoderDrive(.3, 50, 50, 10, false);

        // Push the foundation in a bit more, and park robot.
        // The directions we do this in depend on which side we are on.
        double dir;
        if(blueSide){
            dir=-1.0;

        } else {
            dir=1.0;
        }

        // push the foundation in a bit more
        robot.setStrafePower(.7 * dir);
        sleep(400);

        // park under the sky bridge
        robot.setStrafePower(-.7*dir);
        sleep(1200);
        robot.setPower(0);

    }

}
