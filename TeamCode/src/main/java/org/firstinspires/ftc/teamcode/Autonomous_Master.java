package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 */

public class Autonomous_Master extends Competition_Hardware {


    public void runOpMode() throws InterruptedException {

        initSystem();

        // Wait for the game to start (driver presses PLAY)
        idle();
        waitForStart();
        encoderDrive(DRIVE_SPEED, "up", 10, 5);



    }

public void initSystem(){

    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    // Send telemetry message to indicate successful Encoder reset
    telemetry.addData("motor1", motor1.getCurrentPosition());
    telemetry.addData("motor2", motor2.getCurrentPosition());
    telemetry.addData("motor3", motor3.getCurrentPosition());
    telemetry.addData("motor4", motor4.getCurrentPosition());
}





}
