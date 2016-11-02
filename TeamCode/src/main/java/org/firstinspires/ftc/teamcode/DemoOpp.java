/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.DemoHardware;

/**
 * This is just for running demos
 * This will run the pictching machine
 */

@TeleOp(name="Pushbot: DemoOpp", group="Pushbot")

//@Disabled
public class DemoOpp extends LinearOpMode {

    /* Declare OpMode members. */
    DemoHardware   robot           = new DemoHardware();



    @Override

    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets power to motoros to x stick
            // motor2 is reverse of motor1

            if (gamepad1.dpad_up){
                if (gamepad1.dpad_left){
                    robot.drive("up left");
                    telemetry.addData("gamepad1","dpad up left");
                }else if (gamepad1.dpad_right){
                    robot.drive("up right");
                    telemetry.addData("gamepad1","dpad up right");
                }else{
                    robot.drive("up");
                    telemetry.addData("gamepad1","dpad up");
                }
            }else if (gamepad1.dpad_down){
                if (gamepad1.dpad_left){
                    robot.drive("down left");
                    telemetry.addData("gamepad1","dpad down left");
                }else if (gamepad1.dpad_right){
                    robot.drive("down right");
                    telemetry.addData("gamepad1","dpad down right");
                }else{
                    robot.drive("down");
                    telemetry.addData("gamepad1","dpad down");
                }
            }else if(gamepad1.dpad_left){
                robot.drive("left");
                telemetry.addData("gamepad1","dpad left");
            }else if (gamepad1.dpad_right){
                robot.drive("right");
                telemetry.addData("gamepad1","dpad right");
            }else{
                if (gamepad1.left_trigger > 0){
                    telemetry.addData("gamepad1","left trigger");
                    robot.drive("circle left");
                }else if (gamepad1.right_trigger > 0){
                    telemetry.addData("gamepad2","right trigger");
                    robot.drive("circle right");
                }else{
                    if (gamepad1.y){
                        telemetry.addData("gamepad1","y = 1");
                        robot.motor1.setPower(1);
                        robot.motor2.setPower(0);
                        robot.motor3.setPower(0);
                        robot.motor4.setPower(0);


                    }else if (gamepad1.b){
                        telemetry.addData("gamepad1","b = 2");
                        robot.motor1.setPower(0);
                        robot.motor2.setPower(1);
                        robot.motor3.setPower(0);
                        robot.motor4.setPower(0);

                    }else if(gamepad1.a){
                        telemetry.addData("gamepad1","a =3");
                        robot.motor1.setPower(0);
                        robot.motor2.setPower(0);
                        robot.motor3.setPower(1);
                        robot.motor4.setPower(0);
                    }else if (gamepad1.x){
                        telemetry.addData("gamepad1","x = 4");
                        robot.motor1.setPower(0);
                        robot.motor2.setPower(0);
                        robot.motor3.setPower(0);
                        robot.motor4.setPower(1);
                    }else{
                        telemetry.addData("gamepad1","none");
                        robot.motor1.setPower(0);
                        robot.motor2.setPower(0);
                        robot.motor3.setPower(0);
                        robot.motor4.setPower(0);
                    }
                }

            }


            telemetry.addData("motor1",   "%.2f", robot.motor1.getPower());
            telemetry.addData("motor2",   "%.2f", robot.motor2.getPower());
            telemetry.addData("motor3",   "%.2f", robot.motor3.getPower());
            telemetry.addData("motor4",   "%.2f", robot.motor4.getPower());
            telemetry.addData("Status", "Driving");    //

            updateTelemetry(telemetry);


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
