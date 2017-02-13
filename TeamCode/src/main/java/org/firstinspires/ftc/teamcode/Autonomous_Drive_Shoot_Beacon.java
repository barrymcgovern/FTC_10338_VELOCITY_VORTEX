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




public class Autonomous_Drive_Shoot_Beacon extends Competition_Hardware {
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
                pMotor1.setPower(SPIN_SPEED);
                pMotor2.setPower(-SPIN_SPEED);

                encoderDrive(DRIVE_SPEED, "right", 8, 11);

                runtime.reset();
                while (runtime.seconds() < 2) {
                    beMotor.setPower(-ELEVATOR_SPEED);

                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);
                sleep(100);


                if (teamColor == "blue") {
                    // need to do 180 for blue
                    encoderDrive(DRIVE_SPEED, "circle right", 12.5, 10);
                }

                // goal is to drive pretty close to wall, but not so close as to hit beacon if crooked

                encoderDrive(DRIVE_SPEED, "down", 2, 10);

                if (teamColor == "blue"){
                    encoderDrive(DRIVE_SPEED, "left", 5, 10);
                } else{
                    encoderDrive(DRIVE_SPEED, "right", 5, 10);
                }



                encoderDriveRange(DRIVE_SPEED,"down",22);

                if (teamColor == "blue"){
                    encoderDriveWhiteLine(.2,"left");
                } else {
                    encoderDriveWhiteLine(.2, "right");
                }

                encoderDriveLineUp(.2);
                encoderDriveRange(DRIVE_SPEED, "down", 5);
                // back up just a bit
                encoderDrive(DRIVE_SPEED, "up", 2, 10);

                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();

                //if the color is red
                detectAndActiveBeaconColor();


                sleep(10);
                //Move over to second beacon

                if (teamColor == "blue"){
                    encoderDrive(DRIVE_SPEED, "left", 2, 10);
                    encoderDriveWhiteLine(.2,"left");

                } else {
                    encoderDrive(DRIVE_SPEED, "right", 2, 10);
                    encoderDriveWhiteLine(.2, "right");
                }
                encoderDriveLineUp(.2);
                encoderDriveRange(DRIVE_SPEED, "down", 5);
                // back up just a bit
                encoderDrive(DRIVE_SPEED, "up", 2, 10);



                telemetry.addData("Red", colorSensor.red());
                telemetry.addData("Blue", colorSensor.blue());
                telemetry.update();

                //if the color is red
               detectAndActiveBeaconColor();
                //Turn to prepare for angled drive to center vortex
                if (teamColor == "blue"){
                    encoderDrive(DRIVE_SPEED, "circle right", 2, 10);
                    //moves backwards and knocks cap ball off and parks on center vortex

                } else {
                    encoderDrive(DRIVE_SPEED, "circle left", 2, 10);
                }
                encoderDrive(DRIVE_SPEED,"up", 17, 14);

                break;


            }




        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

    public void runDriveShootBeaconNoEncoder(){
        try {
            /*
          This is going to use all drive commands, no encoder
          will use time, distance, or white line to stop robot

             */

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            sleep(10);
            resetEncoders("drive");
            runtime.reset();

            waitForStart();


            while (opModeIsActive()) {

                // start spinning pwheels immediately to get them up to spped
                pMotor2.setPower(-SPIN_SPEED);
                pMotor1.setPower(SPIN_SPEED);

                runtime.reset();

                speed = .2;
                // 1 second, then shoot
                drive("left", 2);

                drive("stop");
                runtime.reset();
                while (runtime.seconds() < 2) {
                    // shoot the balls
                    beMotor.setPower(-ELEVATOR_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);

                // move a little forward so we miss ramp coming down
                drive("left", 2);


                // 1 second turn
                if (teamColor == "blue") {
                    // need to do 180 for blue
                    drive("circle left", 3);
                }

                drive("stop");
                // goal is to drive pretty close to wall
                // need both sensors to hit white line
                // but not so close as to hit beacon or ramp
                speed = .2;
                while (rangeSensor.rawUltrasonic() > 22){
                    drive("down");
                }

                drive("stop");

                // then go to white line

                gotoWhiteLineAndHitBeacon();

                detectAndActiveBeaconColor();

                // go to 2nd beacon
                drive("right", 1); // first get off 1st beacon white line

                gotoWhiteLineAndHitBeacon();

                detectAndActiveBeaconColor();

                //Turn to prepare for angled drive to center vortex
                //moves backwards and knocks cap ball off and parks on center vortex
                speed = .3;
                if (teamColor == "blue"){
                    drive("circle left", .5);
                    drive("up",4);

                } else {
                    drive("circle right", .5);
                    drive("up",4);
                }

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
            telemetry.addData("ods",ods.getRawLightDetected());
            telemetry.addData("ods2",ods2.getRawLightDetected());
            telemetry.update();
            waitForStart();


            while (opModeIsActive()) {

                speed = .1;
                while ( ods.getRawLightDetected() < .1 && ods2.getRawLightDetected() < .1){
                    telemetry.addData("ods", ods.getLightDetected());
                    telemetry.addData("ods2", ods2.getLightDetected());
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
    public void runDriveBeaconOnly() {
        try {
            while (opModeIsActive()){

                speed = .2;
                // 1 second, then shoot
                drive("right", 2);
                speed = .2;
                while (rangeSensor.rawUltrasonic() > 22) {
                    drive("down");
                }

                drive("stop");

                // then go to white line

                gotoWhiteLineAndHitBeacon();

                detectAndActiveBeaconColor();

                // go to 2nd beacon
                drive("right", .25); // first get off 1st beacon white line

                gotoWhiteLineAndHitBeacon();

                detectAndActiveBeaconColor();

                //Turn to prepare for angled drive to center vortex
                //moves backwards and knocks cap ball off and parks on center vortex
                speed = .3;
                if (teamColor == "blue") {
                    drive("circle right", .25);
                    drive("up", 4);

                } else {
                    drive("circle left", .25);
                    drive("up", 4);
                }

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
            Drives forward, shoots, then knocks cap ball off

             */

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();

            waitForStart();


            while (opModeIsActive()) {

                pMotor1.setPower(SPIN_SPEED);
                pMotor2.setPower(-SPIN_SPEED);
                encoderDrive(DRIVE_SPEED, "right", 8, 11);
                runtime.reset();
                while (runtime.seconds() < 3) {
                    beMotor.setPower(-ELEVATOR_SPEED);
                }
                beMotor.setPower(0);
                pMotor1.setPower(0);
                pMotor2.setPower(0);

                encoderDrive(DRIVE_SPEED, "right", 12, 11);

                break;

            }




        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }



    public void runDriveOnly(){
        try {

            // drives forward only

            init(hardwareMap);

            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();

            waitForStart();


            while (opModeIsActive()) {

              encoderDrive(DRIVE_SPEED, "right", 16, 11);
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
            //gyro.calibrate();


            telemetry.addData("Status","end of init");
            telemetry.update();



        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }
}








