package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by kids on 12/8/2016.
 */
@Autonomous(name="Comp: Blue Beacon", group="Pushbot")

public class Autonomous_Blue_Beacon extends Autonomous_Master{
    @Override
    public void runOpMode() throws InterruptedException {
       while (opModeIsActive()){
           teamColor = "blue";
           runMaster();
       }
    }
}
