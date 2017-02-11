package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by kids on 2/8/2017.
 */
//@Autonomous (name= "Comp: blue Drive Beacon Only", group="Pushbot")
public class Autonomous_Drive_Beacon_Only_Blue extends Autonomous_Drive_Shoot_Beacon {
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";
        runDriveShootBeacon();

    }
}

