package org.firstinspires.ftc.teamcode;

/**
 * Created by Barry on 9/29/2016.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class DemoHardwareKendall {
    /* basic hardware for deom */
    public DcMotor motor1  = null;
    public DcMotor motor2  = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;


    public Servo servo1    = null;
    public Servo servo2   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motor1   = hwMap.dcMotor.get("motor1");
        motor2  = hwMap.dcMotor.get("motor2");
        motor3   = hwMap.dcMotor.get("motor3");
        motor4  = hwMap.dcMotor.get("motor4");


        // Set to FORWARD
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);


        // Set all motors to run without encoders.
        // only use RUN_USING_ENCODERS in autonomous
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    /*
        // Define and initialize ALL installed servos.
        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);
        */
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void drive(String robotDirection){
        try{


        if (robotDirection == "up") {
            motor1.setPower(-1);
            motor2.setPower(1);
            motor3.setPower(1);
            motor4.setPower(-1);

        }else if (robotDirection == "up left"){
            motor1.setPower(0);
            motor2.setPower(1);
            motor3.setPower(0);
            motor4.setPower(-1);

        }else if (robotDirection == "up right"){
            motor1.setPower(0);
            motor2.setPower(-1);
            motor3.setPower(0);
            motor4.setPower(1);

        }else if (robotDirection == "down"){
            motor1.setPower(1);
            motor2.setPower(-1);
            motor3.setPower(-1);
            motor4.setPower(1);

        }else if (robotDirection == "down left"){
            motor1.setPower(1);
            motor2.setPower(0);
            motor3.setPower(-1);
            motor4.setPower(0);
        }else if (robotDirection == "down right"){
            motor1.setPower(-1);
            motor2.setPower(0);
            motor3.setPower(1);
            motor4.setPower(0);

        }else if (robotDirection == "left"){
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(-1);
            motor4.setPower(-1);
        }else  if (robotDirection == "right"){
            motor1.setPower(-1);
            motor2.setPower(-1);
            motor3.setPower(1);
            motor4.setPower(1);

        }else  if (robotDirection == "circle left"){
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(1);
            motor4.setPower(1);

        }else  if (robotDirection == "circle right"){
            motor1.setPower(-1);
            motor2.setPower(-1);
            motor3.setPower(-1);
            motor4.setPower(-1);

        }else{
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
        }  catch (Exception p_exception) {
         //   telemetry.addData("98", "drive error" + p_exception.toString());
        }

    }

}
