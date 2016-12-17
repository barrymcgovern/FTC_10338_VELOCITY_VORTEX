package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Runs Autonomous_Drive_and_Shoot with teamColor set to red for correct directions
@Autonomous (name= "Comp: Red_Drive_Shoot", group="pushbot")
public class Drive_Shoot_Red extends Autonomous_Drive_and_Shoot {

    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";

        runDriveShoot();}

}
