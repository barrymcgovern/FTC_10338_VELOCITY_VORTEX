package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 */
@Autonomous(name="Demo: EncoderTesting", group="Pushbot")

public class EncoderTesting extends Competition_Hardware {

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
                telemetry.addData("Status", "Running");

                telemetry.addData("motor1", motor1.getCurrentPosition());
                telemetry.addData("motor2", motor2.getCurrentPosition());
                telemetry.addData("motor3", motor3.getCurrentPosition());
                telemetry.addData("motor4", motor4.getCurrentPosition()); // Send telemetry message to indicate successful Encoder reset
                telemetry.update();
                encoderDrive(DRIVE_SPEED, "up", 10, 5); // Has the robot go forward at a set speed, in a set direction, for 10 inches with a 5 second timeout

                telemetry.addData("motor1", motor1.getCurrentPosition());
                telemetry.addData("motor2", motor2.getCurrentPosition());
                telemetry.addData("motor3", motor3.getCurrentPosition());
                telemetry.addData("motor4", motor4.getCurrentPosition()); // Send telemetry message to indicate successful Encoder reset
                telemetry.update();
                break;

            }

            telemetry.addData("Status", "Completed");
            telemetry.update();


        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

    public void initSystem() {
        try {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// At the beginning of the program, the encoders on the motors are reset

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // At the beginning of the program, the motors are set to run using encoders until programmed otherwise
        } catch (Exception e) {
            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}
