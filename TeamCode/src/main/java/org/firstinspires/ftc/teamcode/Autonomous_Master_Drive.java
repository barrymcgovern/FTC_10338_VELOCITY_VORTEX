package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.adafruit.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * Autonomous program for knocking the Cap Ball and parking the robot on the Center Vortex base in the event that the other
 * autonomous programs cannot be used.
 * 10 points total
 * Knocks off cap ball for 5 points
 * Partially parks on base
 */
//@Autonomous(name="Comp: Autonomous_Master_Drive", group="Pushbot")
        public class Autonomous_Master_Drive extends Competition_Hardware {
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
                //Uses the encoders to move robot 16 centimeters depending on alliance color
                if (teamColor == "blue") {
                    encoderDrive(DRIVE_SPEED, "left", 16, 10);
                } else {
                    encoderDrive(DRIVE_SPEED, "right", 16, 10);
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
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("2", "MotorTest", motor1.getCurrentPosition());


        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}



