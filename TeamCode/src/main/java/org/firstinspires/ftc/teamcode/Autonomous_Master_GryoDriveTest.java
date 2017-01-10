package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * This is a test of gyroDrive code
 * should drive a square  - up , left, down, right
 */

@Autonomous(name="TEST: Gryo Drive Test", group="Pushbot")

public class Autonomous_Master_GryoDriveTest extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void runDrive(){

        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();


            initSystem(); // See initSystem below

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            while (opModeIsActive()) {
               gyroDrive("up",DRIVE_SPEED,10);
               gyroDrive("left", DRIVE_SPEED, 10);
                gyroDrive("down",DRIVE_SPEED,10);
               gyroDrive("right", DRIVE_SPEED, 10);

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

            telemetry.addData("1", "Servo1_Position", servo1.getPosition());
            telemetry.addData("2", "MotorTest", motor1.getCurrentPosition());


        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}



