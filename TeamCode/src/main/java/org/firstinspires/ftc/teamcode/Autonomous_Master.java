package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by kids on 11/3/2016.
 */

public class Autonomous_Master extends Competition_Hardware {
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public void runOpMode() throws InterruptedException {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("motor1", motor1.getCurrentPosition());
        telemetry.addData("motor2", motor2.getCurrentPosition());
        telemetry.addData("motor3", motor3.getCurrentPosition());
        telemetry.addData("motor4", motor4.getCurrentPosition());


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }







}
