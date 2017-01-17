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
 * 30 points for claiming a beacon
 * 5 points for knocking cap ball off base
 * 5 points for parking partially on base
 *
 *///Runs the Autonomous_Master but with teamColor as red so the directions match red alliance
@Autonomous(name="Comp: Red Beacon Only", group="Pushbot")

public class Autonomous_Red_Beacon extends Autonomous_Drive_and_Shoot{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";
           // only get red beacon
    }
}
