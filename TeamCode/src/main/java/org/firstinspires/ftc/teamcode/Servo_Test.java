package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by megan_000 on 11/28/2016.
 */
@TeleOp(name="Comp: Servo Test", group= "Pushbot")

public class Servo_Test extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException{
        init(hardwareMap);


        telemetry.update();
        waitForStart();


        while (opModeIsActive()){

            //A program that will control driving direction and speed using left joystick


            if (gamepad2.right_stick_y < 0) {
                servo1.setPosition(0);
            }else if (gamepad2.right_stick_y > 0){
                servo1.setPosition(1);
            } else {
                servo1.setPosition(.5);

            }
            telemetry.addData("right_stick_y",gamepad2.right_stick_y);
            telemetry.addData("servo1",servo1.getPosition());
            telemetry.update();
            }
        }
    }


