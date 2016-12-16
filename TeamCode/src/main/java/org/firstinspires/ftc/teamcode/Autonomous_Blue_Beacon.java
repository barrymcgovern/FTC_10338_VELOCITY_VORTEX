package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *
 /**
 * Team: Dark Matters #10338
 * Velocity Vortex Competition
 *
 * Competition Op Mode
 * Our highest scoring program specific to gaining blue points.
 * Scores 40 points at most
 *
 *
 *
 *
 *
 *
 */
// Runs all of existing main autonomous. No changes had to be made in this program.
@Autonomous(name="Comp: Blue Beacon", group="Pushbot")

public class Autonomous_Blue_Beacon extends Autonomous_Master{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

           runMaster();

    }
}
