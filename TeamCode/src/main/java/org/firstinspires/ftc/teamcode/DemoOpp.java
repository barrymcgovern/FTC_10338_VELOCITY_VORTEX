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
    DemoHardware   robot           = new DemoHardware();              // Use a K9'shardware
    String robotDirection;


    @Override
    public void runOpMode() throws InterruptedException {
        double motorPower;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets power to motoros to x stick
            // motor2 is reverse of motor1

            if (gamepad1.dpad_up){
                robot.motor1.setPower(1);
                robot.motor2.setPower(-1);
                robot.motor3.setPower(1);
                robot.motor4.setPower(-1);

            }else if (gamepad1.dpad_down){
                robot.motor1.setPower(-1);
                robot.motor2.setPower(1);
                robot.motor3.setPower(-1);
                robot.motor4.setPower(1);

            }else if(gamepad1.dpad_left){
                robot.motor1.setPower(-1);
                robot.motor2.setPower(1);
                robot.motor3.setPower(-1);
                robot.motor4.setPower(1);
            }else if (gamepad1.dpad_right){
                robot.motor1.setPower(-1);
                robot.motor2.setPower(1);
                robot.motor3.setPower(-1);
                robot.motor4.setPower(1);
            }else{
                robot.motor1.setPower(0);
                robot.motor2.setPower(0);
                robot.motor3.setPower(0);
                robot.motor4.setPower(0);
            }



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
