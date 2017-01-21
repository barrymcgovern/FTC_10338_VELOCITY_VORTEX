package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Runs Autonomous_Drive_and_Shoot with teamColor set to blue for correct directions
@Autonomous(name="Comp: Blue Drive Shoot Beacon", group="Pushbot")
public class Autonomous_Drive_Shoot_Beacon_Blue extends Autonomous_Drive_Shoot_Beacon {
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

        runDriveShootBeacon();

    }

}
