package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by megan_000 on 12/16/2016.
 */
@Autonomous(name="Comp: Red_Drive", group="Pushbot")
public class Drive_Red extends Autonomous_Master_Drive {
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";

        runDrive();
    }
}
