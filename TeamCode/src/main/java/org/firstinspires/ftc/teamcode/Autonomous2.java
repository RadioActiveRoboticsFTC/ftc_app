package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Directions:
// place robot with back against wall to right of sky bridge

// Points:
// 5 points for parking the robot directly under the sky bridge

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)
@Autonomous(name="AutonomousLeft2")
public class Autonomous2 extends BaseAutonomous {

    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'override' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {
        // close claws so that linear sliders fit under sky bridge
        robot.closeClaws();
        sleep(2000);
        driveStraight(0.5, 24.0, 15.0);
        spinLeftP(90,0.5);
        driveStraight(0.5,20,15);
    }
}