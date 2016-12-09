package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by kids on 12/8/2016.
 */
@Autonomous(name="Comp: Red Beacon", group="Pushbot")

public class Autonomous_Red_Beacon extends Autonomous_Master{
    @Override
    public void runOpMode() throws InterruptedException {
       while (opModeIsActive()){
           teamColor = "red";
           runMaster();
       }
    }
}
