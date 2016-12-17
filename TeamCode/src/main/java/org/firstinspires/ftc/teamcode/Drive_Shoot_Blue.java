package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Runs Autonomous_Drive_and_Shoot with teamColor set to blue for correct directions
@Autonomous(name="Comp: Blue_Drive_Shoot", group="Pushbot")
public class Drive_Shoot_Blue extends Autonomous_Drive_and_Shoot {
    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "blue";

        runDriveShoot();

    }

}
