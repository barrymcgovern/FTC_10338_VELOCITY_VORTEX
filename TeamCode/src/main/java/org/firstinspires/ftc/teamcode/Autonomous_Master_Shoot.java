package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 *
 */
@Autonomous(name="Comp: Autonomous_Master", group="Pushbot")

public class Autonomous_Master_Shoot extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();


            initSystem(); // See initSystem below

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < 5) {
                    beMotor.setPower(ELEVATOR_SPEED);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);


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









            } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }





}
