package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * This is a test of gyroDrive code
 * should drive a square  - up , left, down, right
 */

//@Autonomous(name="Comp: GryroDriveTest", group="Pushbot")


public class DemoGryoDriveTest extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            telemetry.addData("Status", "Starting");    //
            telemetry.update();
            init(hardwareMap);

            telemetry.addData("Status", "InitStarted");    //
            telemetry.update();
            initSystem(); // See initSystem below

            telemetry.addData("Status", "Init Done");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)

            // Wait for the game to start (Display Gyro value), and reset gyro before we move..
            while (!isStarted()) {
                telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }


            while (opModeIsActive()) {

                gyroDrive("left", DRIVE_SPEED, 10);
                gyroDrive("down",DRIVE_SPEED,10);
                gyroDrive("right", DRIVE_SPEED, 10);
                gyroDrive("up",DRIVE_SPEED,10);

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

            gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (gyro.isCalibrating())  {
                Thread.sleep(50);
                idle();
            }




        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}



