package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Comp: Blue_Drive", group="Pushbot")
public class Drive_Blue extends Autonomous_Master_Drive{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

        runDrive();
    }
}
