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



public class OLDAutonomous_Drive_Shoot_Beacon extends Competition_Hardware {
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

                if (teamColor == "blue") {
                    encoderDrive(DRIVE_SPEED, "left",8, 11);
                } else {
                    encoderDrive(DRIVE_SPEED, "left", 8, 11);

                }

                runtime.reset();
                while (runtime.seconds() < 3) {
                    beMotor.setPower(-50);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);

                if (teamColor == "red") {
                    encoderDrive(DRIVE_SPEED, "circle right", 12.5, 10);
                 /*  gyro.calibrate()

                    while (gyro.isCalibrating()) {
                        Thread.sleep(50);
                        idle();
                    }
                    */
                    //encoderDrive(DRIVE_SPEED, "right", 4, 15);

                }

                if (teamColor == "blue"){

                    encoderDrive(DRIVE_SPEED,"up" , 6, 10);
                    encoderDrive(DRIVE_SPEED, "left", 4, 20);

                } else {
                    encoderDrive(DRIVE_SPEED,"up" , 6, 10);
                    encoderDrive(DRIVE_SPEED, "right", 4, 20);
                    encoderDrive(DRIVE_SPEED, "up", 4, 20);
                }



                speed = DRIVE_SPEED;
                //run until sensor is less than 17 cm,  or 6 seconds
/*
                while (rangeSensor.rawUltrasonic() > 15 && (runtime.seconds() < 10)) {
                    telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                    telemetry.update();
                    drive("down");
                }
*/
                drive("stop");

                // Ask the listener for the latest information on where the robot is
                latestLocation = listener.getUpdatedRobotLocation();

                // The listener will sometimes return null, so we check for that to prevent errors
                if (latestLocation != null)
                    lastKnownLocation = latestLocation;

                coordinates = lastKnownLocation.getTranslation().getData();

                robotX = coordinates[0];
                robotY = coordinates[1];
                robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                // Send information about whether the target is visible, and where the robot is
                telemetry.addData("Tracking " + target.getName(), listener.isVisible());

                telemetry.addData("Robot X ", robotX);
                telemetry.addData("Robot Y ", robotY);
                telemetry.update();
                while (rangeSensor.rawUltrasonic() > 9 && (runtime.seconds() < 15)) {

                    // Ask the listener for the latest information on where the robot is
                    latestLocation = listener.getUpdatedRobotLocation();

                    // The listener will sometimes return null, so we check for that to prevent errors
                    if (latestLocation != null)
                        lastKnownLocation = latestLocation;

                    coordinates = lastKnownLocation.getTranslation().getData();

                    robotX = coordinates[0];
                    robotY = coordinates[1];

                    telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                    telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                    telemetry.addData("Robot X ", robotX);
                    telemetry.addData("Robot Y ", robotY);
                    telemetry.update();
                    speed = .1;

                    if (robotY > 550) {
                        speed = .1;
                        drive("right");
                    } else if (robotY < 510) {
                        speed = .1;
                        drive("left");
                    } else {
                        speed = .2;
                        drive("down");
                    }

                }

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();
                //if the color is red
                if (colorSensor.red() > colorSensor.blue()) {

                    if (teamColor == "blue") {
                        encoderDrive(DRIVE_SPEED, "left", 2, 10);
                        encoderDrive(DRIVE_SPEED, "up", 1, 12);


                    } else {
                        encoderDrive(DRIVE_SPEED, "right", 2, 10);
                        encoderDrive(DRIVE_SPEED, "up", 1, 12);

                    }
                    //if the color detected is blue...
                } else {//if (colorSensor.blue() > colorSensor.red()) {

                    if (teamColor == "blue") {
                        encoderDrive(DRIVE_SPEED, "right", 2, 10);
                        encoderDrive(DRIVE_SPEED, "up", 1, 12);
                    } else {
                        encoderDrive(DRIVE_SPEED, "left", 2, 10);
                        encoderDrive(DRIVE_SPEED, "up", 1, 12);
                    }
                }
                //moves backwards
             encoderDrive(DRIVE_SPEED, "down", 20, 14);

                break;

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








