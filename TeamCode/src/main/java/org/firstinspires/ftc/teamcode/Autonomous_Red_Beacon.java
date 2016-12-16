package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 /**
 * Team: Dark Matters #10338
 * Velocity Vortex Competition
 *
 * Competition Op Mode
 * Our highest scoring program specific to gaining red points.
 * Scores 40 points at most
 *
 */
@Autonomous(name="Comp: Red Beacon", group="Pushbot")

public class Autonomous_Red_Beacon extends Autonomous_Master{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";
           runMaster();
    }
}
