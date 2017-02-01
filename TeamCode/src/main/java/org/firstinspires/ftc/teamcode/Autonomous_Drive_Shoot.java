package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by kids on 1/20/2017.
 */
@Autonomous(name="Comp: Drive Shoot Only", group="Pushbot")

public class Autonomous_Drive_Shoot extends Autonomous_Drive_Shoot_Beacon_New {
    @Override
    public void runOpMode() throws InterruptedException {
        runDriveShoot();
    }

}








