/*
Modern Robotics ODS Encoder Example1
Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor named "ods1"

This program can be run without a battery and Power Destitution Module.

View the video about this at https://youtu.be/EuDYJPGOOPI.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ODS Get Light Values", group = "MRI")
public class MRI_ODS_Encoder_1 extends OpMode {

    OpticalDistanceSensor ods1;

    //sensor value between 0 and 1023
    int raw1;
    double odsReadingRaw;

    @Override
    public void init() {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");
    }

    @Override
    public void loop() {

        raw1 = (int) (ods1.getLightDetected()*1023);
        odsReadingRaw = ods1.getRawLightDetected();

        telemetry.addData("ODS1 int light", raw1);
        telemetry.addData("ODS1 raw double light", odsReadingRaw);


    }

    @Override
    public void stop() {

    }

}