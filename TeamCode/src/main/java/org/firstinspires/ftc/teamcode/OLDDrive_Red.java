package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Runs Autonomous_Master_Drive with teamColor set to red for correct directions
//@Autonomous(name="Comp: Red_Drive", group="Pushbot")
public class OLDDrive_Red extends OLDAutonomous_Master_Drive {
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";

        runDrive();
    }
}
