/*
Modern Robotics Color Sensor Active & Passive Example
Created 8/9/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.75
Reuse permitted with credit where credit is due

Configuration:
Optical Distance sensor named "ods"
Left drive train motor named "ml"  (two letters)
Right drive train motor named "mr"
Both motors need encoders

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Sensors: ODS", group="Pushbot")


public class MRI_ODS_Wall_Follow extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Motors
    DcMotor mLeft;
    DcMotor mRight;

    //Raw value is a whole number between 0 and 1023
    static double odsReadngRaw;

    // odsReadinRaw to the power of (-0.5)
    static double odsReadingLinear;

    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        mLeft = hardwareMap.dcMotor.get("motor1");
        mRight = hardwareMap.dcMotor.get("motor2");

        //This program was designed around a robot that uses two gears on each side of the drive train.
        //If your robot uses direct drive between the motor and wheels or there are an odd number of gears, the opposite motor will need to be reversed.
        mRight.setDirection(DcMotor.Direction.REVERSE);
        mLeft.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        mRight.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

        waitForStart();

        while (opModeIsActive()) {

            odsReadngRaw = ODS.getRawLightDetected();                       //update raw value
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);                //calculate linear value

            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
            mRight.setPower(odsReadingLinear * 2);
            mLeft.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 Motor Left", mLeft.getPower());
            telemetry.addData("3 Motor Right", mRight.getPower());

            waitOneFullHardwareCycle();
        }
    }
}//end class