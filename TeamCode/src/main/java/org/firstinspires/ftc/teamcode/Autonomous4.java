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
        boolean blueSide = false;
        //moveFoundation(blueSide);
        double straightSpeed = .5;
        double turnSpeed = .3;
        // in theory, we know how far we need to go,
        // but in practice we are off by 'offset'
        double offset = 2.0;
        double dist = (4*12) - 18 - offset;

        // move up to the foundation so that we are positioned
        // to drop our claws on top of it

        /*
        // the purpose of this next group of code is to start
        // in a legal position, but then end in the middle of the foundation

        driveStraight(straightSpeed,dist/3,20);
        robot.brake(1);

        spinLeftP(45,0.8);

        driveStraight(straightSpeed,(dist/3)+5,20);
        robot.brake(1);

        spinRightP(0,0.8);

        driveStraight(straightSpeed,dist/3 ,20);
        robot.brake(1);
        */
        driveStraight(straightSpeed,dist ,20);


        // close claws so that they will drop down on top of the foundation
        robot.straightClaws();

        sleep(500);

        // go backwards so that the grooves catch on the foundation edge,
        // and we can start dragging the foundation towards the building site.
        driveStraight(0.2,-(dist - 4),20 );

        // this part of the code unattaches the robot from the
        // foundation by lifting linear slider, and opening claws,
        // and letting go of slider
        robot.sliderMotor.setPower(0.5);
        sleep(700);
        robot.openClaws();

        //drop linear slider
        robot.sliderMotor.setPower(0);

        // now we need to get to the other side of the side of the foundation,
        // so we can push it further into the building zone
        /*
        if (blueSide) {
            spinRightP(-90, .8);
        } else {
            spinLeftP(-90, .8);
        }
        */
//        spin(blueSide, -90, .8);
        spinRightP(-90, .8);

        driveStraight(.5,28,10);
        robot.brake(1);

        //make sure the linear slider is all the way down
        // so that we don't drive OVER the foundation
        robot.closeClaws();
        sleep(1000);
        robot.openClaws();

         spinLeftP(0,.8);
        //spin(!blueSide, 0, .8);

        driveStraight(.75, 44,20);
        robot.brake(1);

        spinLeftP(90, .8);
//        spin(!blueSide, 90, .8);

        // 12 is too close to the middle of foundation?
        // 6 barely gets the side
        driveStraight(.75,9, 20);
        robot.brake(1);

        spinLeftP(180, .8);
//        spin(!blueSide, 180, .8);

        // push the foundation into the building zone
        // is too high a power causing it to skid?
        // is it risky to use the gyrosensor, when we are at the 180/-180 angle boundary?
//        driveStraight(.2,50, 20);
        encoderDrive(.2, 50, 50, 10, false);

        // get ourselves to a known orientation
        /*
        double yAngle = robot.getYAxisAngle();
        double targetAngle = 165.0;
        if (yAngle < targetAngle) {
            spinLeftP(targetAngle, .5);
        } else {
            spinRightP(targetAngle, .5);
        }
        */

        /*
        robot.setStrafePower(-.5);
        sleep(1000);
        robot.setPower(0);
        */
        // park!
        robot.setStrafePower(.5);
        sleep(400);
        robot.setStrafePower(-.5);
        sleep(1200);
        robot.setPower(0);

    }
 }
