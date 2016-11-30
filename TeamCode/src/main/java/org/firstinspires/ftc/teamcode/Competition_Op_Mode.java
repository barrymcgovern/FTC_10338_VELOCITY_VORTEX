package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by megan_000 on 11/28/2016.
 */
@TeleOp(name="Comp: Main", group= "Pushbot")

public class Competition_Op_Mode extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException{
        init(hardwareMap);



        telemetry.update();
        telemetry.addData("Motor 1", motor1.getCurrentPosition());
        telemetry.addData("Motor 2", motor2.getCurrentPosition());
        telemetry.addData("Motor 3", motor3.getCurrentPosition());
        telemetry.addData("Motor 4", motor4.getCurrentPosition());

        waitForStart();


        while (opModeIsActive()){

            driveStick(gamepad1.left_stick_x, gamepad1.left_stick_y);

            if (gamepad2.left_bumper){
                servo1.setPosition(0);
            }else if (gamepad2.right_bumper){
                servo1.setPosition(180);
            }

            if (gamepad2.dpad_up){
                beMotor.setPower(100);
            } else if (gamepad2.dpad_down){
                beMotor.setPower(-100);
            } else {
                beMotor.setPower(0);
            }

            if (gamepad2.b){
                pMotor1.setPower(100);
                pMotor2.setPower(-100);
            } else{
                pMotor1.setPower(0);
                pMotor2.setPower(0);
            }
        }
    }

}
