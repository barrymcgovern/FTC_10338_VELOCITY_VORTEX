package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * A lower scoring autonomous created in case another team has a similar main autonomous.
 * Scores 15 points.
 * 5 points for each particle scored-up to 2 that can be stored
 * 5 points for parking partially on the base.
 */




public class Autonomous_Drive_Shoot_Beacon_Gyro extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

    }


    public void runDriveShootBeacon() {
        try {
            /*
            drive towards hoop a foot or so
            shoot balls
            if blue, need to spin 180
            drive towards wall till 10 or so inches away

            drive towards white line and stop at white line
            follow white line and hit beacon
                - stop around 7 or so on ultrasonic sensor
            back up an inch
            read color
            if color does not equal team color,
                then hit button again and back up

            Go to next white line and repeat above


            try and knock cap ball off

             */

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();

            waitForStart();


            while (opModeIsActive()) {

                encoderDrive(DRIVE_SPEED, "right", 8, 11);

                runtime.reset();
                while (runtime.seconds() < 2) {
                    beMotor.setPower(-ELEVATOR_SPEED);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);

                if (teamColor == "blue") {
                    // need to do 180 for blue
                    encoderDrive(DRIVE_SPEED, "circle right", 12.5, 10);
                }

                // goal is to drive pretty close to wall, but not so close as to hit beacon if crooked
                speed = .2;
                resetEncoders("drive");
                while (rangeSensor.rawUltrasonic() > 22 ){
                    drive("down");
                }

                drive("stop");
                // this needs to go to white line
                encoderDrive(DRIVE_SPEED, "left", 7, 10);

                resetEncoders("drive");
                speed = .2;
                while (rangeSensor.rawUltrasonic() > 5) {
                    // drive down till robot hits button
                    drive("down");
                }
                // back up just a bit
                encoderDrive(DRIVE_SPEED, "up", 2, 10);

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();

                //if the color is red
                if (colorSensor.red() > colorSensor.blue()) {

                    if (teamColor == "blue") {
                        while (rangeSensor.rawUltrasonic() > 5) {
                            // drive down till robot hits button
                            drive("down");
                        }
                        encoderDrive(DRIVE_SPEED, "up", 2, 10);

                    }
                 //if the color detected is blue...
                } else if (colorSensor.blue() > colorSensor.red()) {
                    if (teamColor == "red") {
                        while (rangeSensor.rawUltrasonic() > 5) {
                            // drive down till robot hits button
                            drive("down");
                        }
                        encoderDrive(DRIVE_SPEED, "up", 2, 10);

                    }
                }

                encoderDrive(DRIVE_SPEED, "up", 2, 10);
                sleep(10);
                //Move over to second beacon
                encoderDrive(DRIVE_SPEED, "left", 17, 10);

                //repeat of deciding whether to hit beacon once or twice
                resetEncoders("drive");
                speed = .1;
                while (rangeSensor.rawUltrasonic() > 5) {
                    // drive down till robot hits button
                    drive("down");
                }
                speed = .2;
                // back up just a bit
                encoderDrive(DRIVE_SPEED, "up", 2, 10);

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();

                //if the color is red
                if (colorSensor.red() > colorSensor.blue()) {
                    //And alliance color is blue, hit the beacon again
                    if (teamColor == "blue") {
                        while (rangeSensor.rawUltrasonic() > 5) {
                            // drive down till robot hits button
                            drive("down");
                        }
                        encoderDrive(DRIVE_SPEED, "up", 2, 10);

                    }
                    //if the color detected is blue...
                } else if (colorSensor.blue() > colorSensor.red()) {
                    //And the alliance color is red, hit the beacon again
                    if (teamColor == "red") {
                        while (rangeSensor.rawUltrasonic() > 5) {
                            // drive down till robot hits button
                            drive("down");
                        }
                        encoderDrive(DRIVE_SPEED, "up", 2, 10);

                    }
                }
                //Turn to prepare for angled drive to center vortex
                encoderDrive(DRIVE_SPEED, "circle right", 2, 10);
                sleep(10);
                //moves backwards and knocks cap ball off and parks on center vortex
                encoderDrive(DRIVE_SPEED,"up", 17, 14);
                speed = .2;
                break;


            }




        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }


    public void runODS(){
        try {
         /*
            Just testing the gyro right now;

             */

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();
            telemetry.addData("ods",ods.getLightDetected());
            telemetry.update();
            waitForStart();


            while (opModeIsActive()) {

                speed = .2;

                if ( ods.getLightDetected() < .2){
                    telemetry.addData("ods", ods.getLightDetected());
                    drive("left")  ;
                    telemetry.update();
                }
                drive("stop");


                break;

            }


        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }

    }


    public void runDriveShoot(){
        try {

            /*
            Just testing the gyro right now;

             */

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();

            waitForStart();


            while (opModeIsActive()) {


                gyroDrive("right",DRIVE_SPEED,8);
                gyroDrive("down",DRIVE_SPEED,8);

/*
                runtime.reset();
                while (runtime.seconds() < 3) {
                    beMotor.setPower(-ELEVATOR_SPEED);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);
*/
                encoderDrive(DRIVE_SPEED, "right", 8, 11);

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

            telemetry.addData("Status", "inside init");    //
            telemetry.update();

            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10);
            gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (gyro.isCalibrating())  {
                telemetry.addData("Status","calibrating");
                telemetry.update();
                Thread.sleep(50);
                idle();
            }
            telemetry.addData("Status","end of init");
            telemetry.update();



        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }
}








