/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Demo: Range Drive", group="Pushbot")

public class RangeDriveTest extends LinearOpMode {

    /* Declare OpMode members. */
    DemoHardware   robot           = new DemoHardware();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            if (gamepad1.y){
                if ( rangeSensor.rawUltrasonic() > 10){
                    robot.motor1.setPower(.25);
                    robot.motor2.setPower(.25);
                    robot.motor3.setPower(-.25);
                    robot.motor4.setPower(-.25);

                }else{
                    runtime.reset();
                    while (opModeIsActive() && runtime.seconds() < 1) {
                        robot.motor1.setPower(.25);
                        robot.motor2.setPower(.25);
                        robot.motor3.setPower(.25);
                        robot.motor4.setPower(.25);
                    }
                }
            }else{
                robot.motor1.setPower(0);
                robot.motor2.setPower(0);
                robot.motor3.setPower(0);
                robot.motor4.setPower(0);
            }






            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }


}