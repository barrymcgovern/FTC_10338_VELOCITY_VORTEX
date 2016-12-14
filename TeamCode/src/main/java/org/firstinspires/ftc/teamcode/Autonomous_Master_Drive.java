package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 12/5/2016.
 * Autonomous program for knocking the Cap Ball and parking the robot on the Center Vortex base in the event that the other
 * autonomous programs cannot be used.
 */
@Autonomous(name="Comp: Autonomous_Master_Drive", group="Pushbot")
        public class Autonomous_Master_Drive extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();



            initSystem(); // See initSystem below

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            while (opModeIsActive()){
                //Uses the encoders to move robot _____ inches
                encoderDrive(DRIVE_SPEED, "left", 5, 10);
                encoderDrive(DRIVE_SPEED, "up", 5, 10);
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



