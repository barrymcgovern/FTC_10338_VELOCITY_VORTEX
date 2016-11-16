package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 */
@Autonomous(name="Comp: Autonomous", group="Pushbot")

public class Autonomous_Master extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();

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

                // Has the robot go forward at a set speed, in a set direction, for 10 inches with a 5 second timeout
                encoderDrive(DRIVE_SPEED, "up", 10, 5);

                runtime.reset();

                // Motors will run without encoders in next step
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //run until sensor is less than 10 cm,  or 6 seconds

                while (rangeSensor.rawUltrasonic() > 10 && (runtime.seconds() < 6)) {
                    telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                    telemetry.update();
                    // will go left or right, depending on red or blue side
                    drive("left");
                }

                // needs to be inside loop
                // Uses the more specific range sensor to make sure that robot is close enough for color sensor before stopping
                if (rangeSensor.rawOptical() > 1) {
                    drive("left");
                } else {
                    drive("stop");
                }

                drive("stop");

                // will move button pusher
                // will need timeout and then move back to nuetral afterwards
                if (colorSensor.red() > colorSensor.blue()) {
                    telemetry.addData("1", "Red", colorSensor.red());
                    servo1.setPosition(90);
                } else {
                    servo1.setPosition(0);
                }
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

        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}
