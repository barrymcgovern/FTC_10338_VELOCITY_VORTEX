package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 * This autonomous will use the Pitching machine and human aim to propel particles into the Center Vortex in the event
 * that the other autonomous programs cannot be used.
 */
@Autonomous(name="Comp: Autonomous_Master_Shoot", group="Pushbot")

public class Autonomous_Master_Shoot extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            //Initializes motors
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();


            initSystem(); // See initSystem below

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < 5) {
                //Ball elevator is powered feeding Pitching Machine with particles
                //Will stop motors after 5 seconds of running
                    beMotor.setPower(100);
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
