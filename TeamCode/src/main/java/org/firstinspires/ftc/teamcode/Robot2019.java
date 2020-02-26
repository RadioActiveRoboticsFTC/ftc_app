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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */

public class Robot2019
{

    // motors for our chassis & drive train
    public DcMotor leftDrive   = null;
    public DcMotor rightDrive  = null;
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;

    //Linear slider motors
    public DcMotor leftSliderMotor = null;
    public DcMotor rightSliderMotor = null;

    // these are the servos for the claw
    public Servo leftServo    = null;
    public Servo rightServo   = null;

    // TBF: we are no longer using these
    public Servo rearRightServo = null;
    public Servo rearLeftServo = null;

    // this is the servo for tipping over the capstone
    public Servo capServo = null;

    /*
    This section is for the hardcoded positions of the various servos
     */
    float raisedPositionCap = (float) 0.44;
    float dropPositionCap = (float)   0.69;

    float closedPositionL = (float) .71;
    float straightPositionL = (float) .52;
    float openPositionL = (float) .475;
    float triggerDownL = (float) 1.0;

    float closedPositionR = (float) .47;
    float openPositionR = (float) .75;
    float straitPositionR = (float) .67;

    // these are for the gamepads, so shouldn't be in here?
    float triggerDownR = (float) 1.0;
    float gearTriggerDown = (float) 0.7;
    float strafeTriggerDown =  (float) 0.70;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    // We'll use this for initializing all hardware
    HardwareMap hwMap  =  null;

    /* Constructor */
    public Robot2019(){

    }

    /* Initialize standard Hardware interfaces
    *  We got this idea from some of the example code */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // the names below represent the names in the configuration file
        // 'Robot2019'

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");

        // Slider Motors
        leftSliderMotor = hwMap.get(DcMotor.class, "slider_motor");
        rightSliderMotor = hwMap.get(DcMotor.class, "slider_motor2");
        leftSliderMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSliderMotor.setDirection(DcMotor.Direction.FORWARD);

        // The drive motors on each side are mirrors of one another.  So if
        // we want them to run in the same direction given the same power, one
        // side needs to be defined to go in the direction opposite of the other
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setPower(0);

        // Use encoders by default
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        // We got this code from the examples.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /* Initialize the servos */
        leftServo = hwMap.get(Servo.class, "LeftServo");
        rightServo = hwMap.get(Servo.class, "RightServo");

        rearLeftServo = hwMap.get(Servo.class, "RearLeftServo");
        rearRightServo = hwMap.get(Servo.class, "RearRightServo");

        capServo = hwMap.get(Servo.class, "CapServo");

    }

    // This gets the servo in its starting position
    public void raiseRearServos() {
        rearRightServo.setPosition(0);
        rearLeftServo.setPosition(0.9);
    }

    // This gets the servo to its clamped position
    public void lowerRearServos() {
        rearRightServo.setPosition(0.7);
        rearLeftServo.setPosition(0.3);
    }

    // sets power to all chassis motors
    public void setPower(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

    // set power to the motors on each side of the chassis indiviually
    public void setPower(double leftPower, double rightPower){
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftFrontDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
    }

    // set drive mode for all motors in the chassis
    public void setDriveMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
    }

    // set target position for each motor in the chassis
    public void setTargetPosition(int newLeftTarget, int newRightTarget){
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftFrontDrive.setTargetPosition(newLeftTarget);
        rightFrontDrive.setTargetPosition(newRightTarget);
    }

    // strafe to the left.  we don't have to worry about the sign of the power
    public void setLeftStrafePower(double power) {
        setStrafePower(-Math.abs(power));
    }

    // strafe to the right.  we don't have to worry about the sign of the power
    public void setRightStrafePower(double power) {
        setStrafePower(Math.abs(power));
    }

    // to strafe, twin drives must be set to opposing powers.
    // strafeing left or right depends on sign of power
    public void setStrafePower(double power) {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
    }

    // get the gyrosenor angle.  Straight ahead is zero.
    // to the left is positive until 180, then it turns into -180.
    // to the right is negative, until -180, then it turns into 180.S
    public double getYAxisAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    // function to brake the robot, set the direction to 1 or -1,
    // 1 for braking after going forward, -1 for backward
    // TBF: create brakeForward and brakeBackwards functions that call this
    public void brake(double direction){
        setPower(-.2*direction);
        //sleep(10);
        setPower(0);
    }

    // move servo so that capstone stays in robot
    public void raiseCap() {
        capServo.setPosition(raisedPositionCap);
    }

    // move servo so that capstone gets dumped
    public void dropCap() {
        capServo.setPosition(dropPositionCap);
    }

    // Move the servos so the claws can grab a brick
    // TBF: why do these use the 'openPosition' variables?
    public void closeClaws(){
        leftServo.setPosition(openPositionL);
        rightServo.setPosition(openPositionR);
    }

    // Move the servos so that the claws can grab the foundation
    public void straightClaws(){
        leftServo.setPosition(straightPositionL);
        rightServo.setPosition(straitPositionR);
    }

    // Move the servos so that the claws are open
    // as wide as possible
    public void openClaws(){
        rightServo.setPosition(closedPositionR);
        leftServo.setPosition(closedPositionL);
    }

    // sets the power of both linear slider motors
    public void setLinearMotorPower(double power){
        leftSliderMotor.setPower(power);
        rightSliderMotor.setPower(power);
    }

}


