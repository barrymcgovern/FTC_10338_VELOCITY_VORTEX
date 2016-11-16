package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 */
@Autonomous(name="Demo: Autonomous", group="Pushbot")

public class Autonomous_Master extends Competition_Hardware {


    public void runOpMode() throws InterruptedException {

        initSystem(); // See initSystem below

        // Wait for the game to start (driver presses PLAY)
        idle();
        waitForStart();
        encoderDrive(DRIVE_SPEED, "up", 10, 5); // Has the robot go forward at a set speed, in a set direction, for 10 inches with a 5 second timeout
        runtime.reset();
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Motors run without encoders to regulate distance
        while (rangeSensor.rawUltrasonic() > 10 && (runtime.seconds() < 6)){
            telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
            drive("left"); // Goes forward then after a certain distance it goes left until certain distance then stops
        }
        if (rangeSensor.rawOptical() > 1){
            drive("left");
        } else {
            drive("stop");
        } // Uses the more specific range sensor to make sure that robot is close enough for color sensor before stopping

        drive("stop");

        if (colorSensor.red() > colorSensor.blue()){
            telemetry.addData("1", "Red", colorSensor.red());
            servo1.setPosition(90);
        }
        else{
            servo1.setPosition(0);
        }
    }

public void initSystem(){

    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// At the beginning of the program, the encoders on the motors are reset


    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // At the beginning of the program, the motors are set to run using encoders until programmed otherwise



    telemetry.addData("motor1", motor1.getCurrentPosition());
    telemetry.addData("motor2", motor2.getCurrentPosition());
    telemetry.addData("motor3", motor3.getCurrentPosition());
    telemetry.addData("motor4", motor4.getCurrentPosition()); // Send telemetry message to indicate successful Encoder reset
}





}
