package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 * The main autonomous program that will claim the beacon for either red or blue
 */


public class Autonomous_Master extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void runMaster(){
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();
            speed = DRIVE_SPEED;

            initSystem(); // See initSystem below

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive()) {

                // Send telemetry message to show position;
                telemetry.addData("motor1", motor1.getCurrentPosition());
                telemetry.addData("motor2", motor2.getCurrentPosition());
                telemetry.addData("motor3", motor3.getCurrentPosition());
                telemetry.addData("motor4", motor4.getCurrentPosition());
                telemetry.update();

                // Has the robot go forward at a set speed, in a set direction, for 14 inches with a 5 second timeout
                encoderDrive(DRIVE_SPEED, "left", 15.5, 5);

                runtime.reset();

                // Motors will run without encoders in next step
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                speed = .2;

                //run until sensor is less than 5 cm,  or 6 seconds

                while (rangeSensor.rawUltrasonic() > 15 && (runtime.seconds() < 6)) {
                    telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                    telemetry.update();
                    // will go left or right, depending on red or blue side
                    if (teamColor == "blue"){
                        drive("down");
                    }
                    else {
                        drive("up");
                    }

                }
                drive("stop");

                // needs to be inside loop
                // Uses the more specific range sensor to make sure that robot is close enough for color sensor before stopping


                // will move button pusher if beacon is red
                // will need timeout and then move back to neutral afterwards
                if (colorSensor.red() > colorSensor.blue()) {
                    telemetry.addData("1", "Red", colorSensor.red());
                    if (teamColor == "blue") {
                        servo1.setPosition(180);
                    } else {
                        servo1.setPosition(0);

                    }
                }
                //will move button pusher if beacon is blue
                if (colorSensor.blue() > colorSensor.red()){
                    telemetry.addData("1", "Blue", colorSensor.blue());
                    if (teamColor == "blue"){
                        servo1.setPosition(0);
                    } else {
                        servo1.setPosition(180);
                    }
                }
                break;

            }

        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

    public void initSystem() {
        try {
            // At the beginning of autonomous program, the encoders on the motors are reset
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            servo1.setPosition(90);

            telemetry.addData("1", "Servo1_Position", servo1.getPosition());
            telemetry.addData("2", "MotorTest", motor1.getCurrentPosition());





        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}
