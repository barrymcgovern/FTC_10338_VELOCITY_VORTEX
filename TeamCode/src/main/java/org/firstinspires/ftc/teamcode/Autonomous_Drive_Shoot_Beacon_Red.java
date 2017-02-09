package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Runs Autonomous_Drive_and_Shoot with teamColor set to red for correct directions
@Autonomous (name= "Comp: RED Drive Shoot Beacon", group="Pushbot")
public class Autonomous_Drive_Shoot_Beacon_Red extends Autonomous_Drive_Shoot_Beacon {

    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";

        runDriveShootBeaconNoEncoder();}

}
