package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Autonomous mode for grabbing foundation and moving it into building zone on blue side

// Directions:
// Place robot with back against wall adjacent to edge of building site on blue side

// Points:  15 points for moving foundation into building site; 5 for parking

// This is an example of creating an autonomous mode based off of
// BaseAutonomous (we 'extend' it)
@Autonomous(name="Autonomous5")
public class Autonomous5 extends BaseAutonomous {
    // The BaseAutonomous class will have it's runOpsMode function called.  It will set up
    // basic stuff for us, but then it calls it's own runAutoOpMode, which does nothing.
    // We 'overrid' runAutoOpMode to do *something*
    @Override
    public void runAutoOpMode() {
        // this works for the blue side
        moveFoundation(true);
    }
}

