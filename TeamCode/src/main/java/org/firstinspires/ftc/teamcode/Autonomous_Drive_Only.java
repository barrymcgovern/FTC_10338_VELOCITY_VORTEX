package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by kids on 1/20/2017.
 */
@Autonomous(name="Comp: Drive Only", group="Pushbot")

public class Autonomous_Drive_Only extends Autonomous_Drive_Shoot_Beacon_New {
    @Override
    public void runOpMode() throws InterruptedException {
        runDriveOnly();
    }

}








