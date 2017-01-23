package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name="Comp: Drive Shoot 2", group="Pushbot")
public class Autonomous_Drive_Shoot_2 extends Competition_Hardware {
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

               encoderDrive(DRIVE_SPEED, "left", 5, 15);
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








