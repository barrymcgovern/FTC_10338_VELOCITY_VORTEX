package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by megan_000 on 12/14/2016.
 */
@Autonomous(name="Comp: Autonomous_Drive_Shoot", group="Pushbot")
public class Autonomous_Drive_and_Shoot extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();
            initSystem();
            while (opModeIsActive() && runtime.seconds() > 5){
                beMotor.setPower(100);
                pMotor1.setPower(SPIN_SPEED);
                pMotor2.setPower(-SPIN_SPEED);

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
