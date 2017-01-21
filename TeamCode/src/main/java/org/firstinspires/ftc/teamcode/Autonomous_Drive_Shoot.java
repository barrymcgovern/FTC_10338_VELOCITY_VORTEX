package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by kids on 1/20/2017.
 */
@Autonomous(name="Comp: Drive Shoot", group="Pushbot")
public class Autonomous_Drive_Shoot extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

    }
        public void runDriveShoot () {
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
                        gyroDrive("left", DRIVE_SPEED, 12);
                    } else {
                        gyroDrive("left", DRIVE_SPEED, 12);

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
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "b4 gyro");    //
            telemetry.update();
            gyro.calibrate();

            while (gyro.isCalibrating()) {
                Thread.sleep(50);
                idle();
            }
            telemetry.addData("Status", "after gyro");    //
            telemetry.update();
            gyro.resetZAxisIntegrator();
            setupVuforia();

            // We don't know where the robot is, so set it to the origin
            // If we don't include this, it would be null, which would cause errors later on
            lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

            visionTargets.activate();
            OpenGLMatrix latestLocation;
            float[] coordinates;


        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
            }
        }
    }









