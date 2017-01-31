package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * A lower scoring autonomous created in case another team has a similar main autonomous.
 * Scores 15 points.
 * 5 points for each particle scored-up to 2 that can be stored
 * 5 points for parking partially on the base.
 */



public class Autonomous_Drive_Shoot_Beacon_New extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void runDriveShootBeacon() {
        try {

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();


            while (opModeIsActive()) {
                encoderDrive(DRIVE_SPEED, "right", 8, 11);

                runtime.reset();
                while (runtime.seconds() < 3) {
                    beMotor.setPower(-50);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);

                if (teamColor == "blue") {
                    encoderDrive(DRIVE_SPEED, "circle right", 12.5, 10);

                }

                encoderDrive(DRIVE_SPEED, "down", 10, 10);
                // this needs to go till white line
                encoderDrive(DRIVE_SPEED, "right", 1.5, 10);

                while (rangeSensor.rawUltrasonic() > 7) {
                    // drive down till robot hits button
                    drive("down");
                }
                encoderDrive(DRIVE_SPEED, "up", .5, 10);

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();
                //if the color is red
                if (colorSensor.red() > colorSensor.blue()) {

                    if (teamColor == "blue") {
                        encoderDrive(DRIVE_SPEED, "down", .5, 10);

                    }
                    //if the color detected is blue...
                } else if (colorSensor.blue() > colorSensor.red()) {

                    if (teamColor == "red") {
                        encoderDrive(DRIVE_SPEED, "down", .5, 10);

                    }
                }
                //moves backwards and knocks cap ball off and parks on center vortex
                encoderDrive(DRIVE_SPEED, "up", 20, 14);

                break;



               /*
               We plan to go forward"left", shoot, go right"back" , and then go forward"left" until
               a sensor can find the white line, then go forward staying on the white line, and hit
               both beacon buttons. Back up slightly, then check to see if color on the beacon
               matches the team color. If so, stop, and if not, hit it again, and back up.
                */


            }




        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

    // need to detect color, move to correct button, bump button
    // then back up and knock ball off stand and drive on stand


    public void initSystem() {
        try {
            // At the beginning of autonomous program, the encoders on the motors are reset

            telemetry.addData("Status", "inside init");    //
            telemetry.update();
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.update();
            gyro.resetZAxisIntegrator();
            setupVuforia();

            // We don't know where the robot is, so set it to the origin
            // If we don't include this, it would be null, which would cause errors later on
            lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

            visionTargets.activate();
            OpenGLMatrix latestLocation;
            float[] coordinates;
            telemetry.addData("Status", "Init Ready");


        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }
}








