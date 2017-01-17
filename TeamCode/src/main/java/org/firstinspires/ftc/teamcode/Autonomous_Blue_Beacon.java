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
 *Scores 40 points at most
 *Scores 30 points for one beacon
 *Scores 5 points for knocking cap ball off the base.
 *Scores 5 points for parking partially on base.
 *
 *
 *
 *
 *
 */
// Runs all of existing main autonomous except teamColor is blue so it runs autonomous for blue alliance
@Autonomous(name="Comp: Blue Beacon Only", group="Pushbot")

public class Autonomous_Blue_Beacon extends Autonomous_Drive_Shoot_Beacon{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

           // only gets blue beacon runDriveShoot();

    }
}
