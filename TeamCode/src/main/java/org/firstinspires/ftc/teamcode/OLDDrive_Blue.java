package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//runs autonomous_Drive with teamColor set to blue for correct directions
//@Autonomous(name="Comp: Blue_Drive", group="Pushbot")
public class OLDDrive_Blue extends OLDAutonomous_Master_Drive{
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

        runDrive();
    }
}
