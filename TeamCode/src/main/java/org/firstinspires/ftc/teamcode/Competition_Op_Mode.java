package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Team: Dark Matters #10338
 * Velocity Vortex Competition
 *
 * Competition Op Mode
 * Our primary program for running tele op mode
 * GamePad 1 usse stick to drive.
 *  left stick drives up down right left
 *  right stick will spin the robot
 * Gamepad 2 operates the elevator and pitching machine
 *
 *
 *
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
        telemetry.update();
        waitForStart();


        while (opModeIsActive()){

            //A program that will control driving direction and speed using left joystick

            if (gamepad1.right_stick_x > 0){
                speed = (gamepad1.right_stick_x);
                drive("circle right");
            } else if (gamepad1.right_stick_x < 0){
                speed = -(gamepad1.right_stick_x);
                drive("circle left");
            } else{
                driveStick(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }


            //Dpad controls pitching machine and ball elevator
            if (gamepad2.left_stick_y < 0){
                beMotor.setPower(-100);

            } else if (gamepad2.left_stick_y > 0){
                beMotor.setPower(100);

            } else {
                beMotor.setPower(0);

            }

            if (gamepad2.right_stick_y < 0){
                servo1.setPosition(1);
                pMotor1.setPower(SPIN_SPEED);
                pMotor2.setPower(-SPIN_SPEED);

            } else {
                servo1.setPosition(.5);
                pMotor1.setPower(0);
                pMotor2.setPower(0);
            }
            telemetry.update();
            }
        }
    }


