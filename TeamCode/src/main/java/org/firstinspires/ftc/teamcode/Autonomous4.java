package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Autonomous mode for grabbing foundation and moving it into building zone

// Directions:

// Points:

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)

@Autonomous(name="Autonomous4")
public class Autonomous4 extends BaseAutonomous {

    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'overrid' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {
        double straightSpeed = .5;
        double turnSpeed = .3;
        // in theory, we know how far we need to go,
        // but in practice we are off by 'offset'
        double offset = 2.0;
        double dist = (4*12) - 18 - offset;

        // move up to the foundation so that we are positioned
        // to drop our claws on top of it

        //robot.setStrafePower(-.3);
        //sleep(500);
        //robot.setPower(0);
        // sleep(5000);
        //robot.brake();
        driveStraight(straightSpeed,dist,20);
        robot.brake(1);

        // close claws so that they will drop down on top of the foundation
        //robot.leftServo.setPosition(robot.openPositionL);
        //robot.rightServo.setPosition(robot.openPositionR);
        robot.closeClaws();

        sleep(1000);
        // go backwards so that the grooves catch on the foundation edge,
        // and we can start dragging the foundation towards the building site.
        driveStraight(0.1,-dist,20 );

        // this part of the code unattaches the robot from the
        // foundation by lifting linear slider, and opening claws,
        // and letting go of slider
        robot.sliderMotor.setPower(1);
        sleep(700);
        //robot.rightServo.setPosition(robot.closedPositionR);
        //robot.leftServo.setPosition(robot.closedPositionL);
        robot.openClaws();
        //drop linear slider
        robot.sliderMotor.setPower(0);

        spinRight(-90,.1);
        driveStraight(.2,28,10);
        spinLeft(0,.1);
        //robot.setStrafePower(.3);
        //sleep(2000);

        driveStraight(straightSpeed, 48,20);
        spinLeft(90, turnSpeed);
        driveStraight(straightSpeed,8, 20);
        spinLeft(180, turnSpeed);
        //make sure the linear slider is all the way down
        //robot.rightServo.setPosition(robot.openPositionR);
        //robot.leftServo.setPosition(robot.openPositionL);
        robot.closeClaws();
        sleep(1000);
        //robot.rightServo.setPosition(robot.closedPositionR);
        //robot.leftServo.setPosition(robot.closedPositionL);
        robot.openClaws();
        driveStraight(straightSpeed,46, 20);
        robot.setStrafePower(-.5);
        sleep(1000);
        robot.setPower(0);





    /*
        robot.setStrafePower(0.3);
    sleep(1000);
    robot.setStrafePower(0);
    driveStraight(.5, -30, 10);
    robot.lowerRearServos();
    sleep(2000);
    driveStraight(.7,24,10);
*/
    /*
        // TBF: just a test
        robot.raiseRearServos();
        sleep(500);
        driveStraight(0.5, -9.0, 15.0);
        robot.lowerRearServos();
        sleep(2000);
        driveStraight(0.5, 9.0, 15.0);

    }
*/
}
 }
