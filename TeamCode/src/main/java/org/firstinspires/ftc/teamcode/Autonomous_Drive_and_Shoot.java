package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * A lower scoring autonomous created in case another team has a similar main autonomous.
 * Scores 15 points.
 * 5 points for each particle scored-up to 2 that can be stored
 * 5 points for parking partially on the base.
 */


//@Autonomous(name="Comp: Autonomous_Drive_Shoot", group="Pushbot")

public class Autonomous_Drive_and_Shoot extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void runDriveShoot(){
        try {
            init(hardwareMap);
            //Pulls ball up with ball elevator and then pitches with a 7 second timeout.
            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            //Within 7 seconds, the 'ball elevator' will feed the balls into the pitching machine to launch them into the Center Vortex
            while (opModeIsActive()){
                while (runtime.seconds() < 7) {
                    beMotor.setPower(-100);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                    //servo1.setPosition(1);

                }
                //if alliance color is blue, the robot will move over and then move to the Center Vortex
                if (teamColor == "blue") {
                    encoderDrive(DRIVE_SPEED, "down", 3, 10);
                    encoderDrive(DRIVE_SPEED, "left", 17, 10);
                //the alternative directions if alliance color is red
                } else {
                    encoderDrive(DRIVE_SPEED, "up", 3, 10);
                    encoderDrive(DRIVE_SPEED, "right", 17, 10);
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
            // Running off encoder numbers
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         //   telemetry.addData("1", "Servo1_Position", servo1.getPosition());
            telemetry.addData("2", "MotorTest", motor1.getCurrentPosition());


        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();

        }
    }
}
