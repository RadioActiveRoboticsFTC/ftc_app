package org.firstinspires.ftc.teamcode;

//import BaseAutonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Directions:
// Place robot against wall, facing the sky bridge, either side.  Have
// the front of the robot almost under the sky bridge. See Diagram ???

// Points:
// 5 points for parking the robot directly under the sky bridge

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)

@Autonomous(name="Autonomous2")
public class Autonomous2 extends BaseAutonomous {

    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'overrid' runAutoOpMode to do *something*

    @Override
    public void runAutoOpMode() {

        // driving straight this distance parks the robot directly under
        // the sky bridge
        driveStraight(0.5, 24.0, 15.0);
        spinLeft(90,0.5);
        driveStraight(0.5,15,15);
    }
}