package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous (name= "Comp: Red_Drive_Shoot", group="pushbot")
public class Drive_Shoot_Red extends Autonomous_Drive_and_Shoot {

    @Override
    public void runOpMode() throws InterruptedException {
        teamColor = "red";

        runDriveShoot();}

}
