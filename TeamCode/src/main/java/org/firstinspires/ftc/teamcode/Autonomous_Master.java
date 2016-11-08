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
        encoderDrive(DRIVE_SPEED, "up", 10, 5); //Has the robot go forward at a set speed, in a set direction, for 10 inches with a 5 second timeout
        runtime.reset();
         while (rangeSensor.rawUltrasonic() > 10 && (runtime.seconds() < 6)){
             telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
             drive("left"); //goes forward then after a certain distance it goes left until certain distance then stops

         }

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
