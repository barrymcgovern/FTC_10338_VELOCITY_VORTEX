package org.firstinspires.ftc.teamcode;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * Hardware definitions
 * 7 dc motors - 4 drive, 2 picthing machine, 1 elevator
 * 1 servo
 * color sensor
 * range finder
 * drive functions
 *  encoderDrive
 *  driveStick
 *  drive
 */


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.security.PublicKey;


public abstract class Competition_Hardware extends LinearOpMode {
    /* basic hardware for demo */
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public DcMotor pMotor1 = null;
    public DcMotor pMotor2 = null;
    public DcMotor beMotor = null;

    double speed;

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    final double SPIN_SPEED = .85;
    final double ELEVATOR_SPEED = .5;

    int newLeftTarget;
    int newRightTarget;
    int moveCounts;
    double max;
    double error;
    double steer;
    double leftSpeed;
    double rightSpeed;

    //public Servo servo1 = null;
    //public Servo servo2 = null;

    ModernRoboticsI2cGyro gyro    = null;
/*
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
*/

    public String teamColor;

    ColorSensor colorSensor;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    ModernRoboticsI2cRangeSensor rangeSensor;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        try{


        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");
        motor4 = hwMap.dcMotor.get("motor4");
        pMotor1 = hwMap.dcMotor.get("pMotor1");
        pMotor2 = hwMap.dcMotor.get("pMotor2");
        beMotor = hwMap.dcMotor.get("beMotor");



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

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.colorSensor.get("color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        // Define and initialize ALL installed servos.
        //servo1 = hwMap.servo.get("servo1");
        runtime.reset();

        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void encoderDrive(double eSpeed, String robotDirection, double inches, double timeoutS) throws InterruptedException {
        try {
            if (opModeIsActive()) {


                int currentPosMotor1 = 0;
                int currentPosMotor2 = 0;
                int currentPosMotor3 = 0;
                int currentPosMotor4 = 0;

                int targetPosMotor1 = 0;
                int targetPosMotor2 = 0;
                int targetPosMotor3 = 0;
                int targetPosMotor4 = 0;

                // Stop all motion;
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);

                // change directions so encoders are all positive
                //Shows all possible directions and controls.
                if (robotDirection == "up") {
                    motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor4.setDirection(DcMotorSimple.Direction.REVERSE);
                } else if (robotDirection == "down") {
                    motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor4.setDirection(DcMotorSimple.Direction.FORWARD);
                } else if (robotDirection == "left") {
                    motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor3.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor4.setDirection(DcMotorSimple.Direction.REVERSE);
                } else if (robotDirection == "right") {
                    motor1.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor2.setDirection(DcMotorSimple.Direction.REVERSE);
                    motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor4.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                //Resets encoders

                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Getting position of each driving motor

                currentPosMotor1 = motor1.getCurrentPosition();
                currentPosMotor2 = motor2.getCurrentPosition();
                currentPosMotor3 = motor3.getCurrentPosition();
                currentPosMotor4 = motor4.getCurrentPosition();

                targetPosMotor1 = currentPosMotor1 + (int) (inches * COUNTS_PER_INCH);
                targetPosMotor2 = currentPosMotor2 + (int) (inches * COUNTS_PER_INCH);
                targetPosMotor3 = currentPosMotor3 + (int) (inches * COUNTS_PER_INCH);
                targetPosMotor4 = currentPosMotor4 + (int) (inches * COUNTS_PER_INCH);

                motor1.setTargetPosition(targetPosMotor1);
                motor2.setTargetPosition(targetPosMotor2);
                motor3.setTargetPosition(targetPosMotor3);
                motor4.setTargetPosition(targetPosMotor4);

                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //sets motors needed to make robot go different directions
                if (robotDirection == "circle left") {
                    motor1.setPower(0);
                    motor2.setPower(eSpeed);
                    motor3.setPower(0);
                    motor4.setPower(eSpeed);

                } else if (robotDirection == "circle right") {

                    motor1.setPower(eSpeed);
                    motor2.setPower(0);
                    motor3.setPower(eSpeed);
                    motor4.setPower(0);
                } else {
                    motor1.setPower(eSpeed);
                    motor2.setPower(eSpeed);
                    motor3.setPower(eSpeed);
                   motor4.setPower(eSpeed);
                }
                // keep looping while we are still active, and there is time left, and both motors are running.

                runtime.reset();

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (motor1.isBusy() && motor2.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Status", "Running Encoder Drive");
                    telemetry.addData("Direction", robotDirection);
                    telemetry.addData("motor1", motor1.getPower());
                    telemetry.addData("motor2", motor2.getPower());
                    telemetry.addData("motor3", motor3.getPower());
                    telemetry.addData("motor4", motor4.getPower());
                    telemetry.update();

                    // Allow time for other processes to run.
                    idle();
                }
                telemetry.addData("Status", "Done with Encoder Drive");
                telemetry.update();

                // Stop all motion;
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);

                // reset directions
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
                motor2.setDirection(DcMotorSimple.Direction.FORWARD);
                motor3.setDirection(DcMotorSimple.Direction.FORWARD);
                motor4.setDirection(DcMotorSimple.Direction.FORWARD);

                // Turn off RUN_TO_POSITION
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
        }
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }




        void driveStick(float x, float y) {

        // speed is greater value of x or y
        //Uses the value of the joystick like the direction of motion does, only to set speed and divides it in half
        speed = (Math.abs(x) > Math.abs(y) ? Math.abs(x) : Math.abs(y))/2;
        telemetry.addData("y", y);
        telemetry.addData("x", x);
        //One program to combine 8 directions of motion on one joystick using ranges of x and y values
        if (y > .10) {
            if (x > .10) {
                drive("up right");
            } else if (x < -.10) {
                drive("up left");

            } else {
                drive("up");

            }
        }else if (y < -.10) {
            if (x > .10) {
                drive("down left");
            } else if (x < -.10) {
                drive("down right");
            } else {
                drive("down");
            }
        } else if (x > .10) {
            drive("right");


        }else if (x < -.10 ){

                drive ("left");

        }else{
            drive("stop");
        }


    }



    void drive(String robotDirection){
        try{

            telemetry.addData("direction",robotDirection);
            //sets speed needed for motors to run different directions
            //Uses four motor to move robot ten different directions
            //Negative speed moves motor backwards and positive speed moves motor forward
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.FORWARD);
            motor3.setDirection(DcMotor.Direction.FORWARD);
            motor4.setDirection(DcMotor.Direction.FORWARD);

            if (robotDirection == "up") {
                motor1.setPower(-speed);
                motor2.setPower(speed);
                motor3.setPower(speed);
                motor4.setPower(-speed);


            }else if (robotDirection == "up left"){
                motor1.setPower(0);
                motor2.setPower(speed);
                motor3.setPower(0);
                motor4.setPower(-speed);

            }else if (robotDirection == "up right"){
                motor1.setPower(0);
                motor2.setPower(-speed);
                motor3.setPower(0);
                motor4.setPower(speed);

            }else if (robotDirection == "down"){
                motor1.setPower(speed);
                motor2.setPower(-speed);
                motor3.setPower(-speed);
                motor4.setPower(speed);

            }else if (robotDirection == "down left"){
                motor1.setPower(speed);
                motor2.setPower(0);
                motor3.setPower(-speed);
                motor4.setPower(0);
            }else if (robotDirection == "down right"){
                motor1.setPower(-speed);
                motor2.setPower(0);
                motor3.setPower(speed);
                motor4.setPower(0);

            }else if (robotDirection == "left"){
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(-speed);
            }else  if (robotDirection == "right"){
                motor1.setPower(-speed);
                motor2.setPower(-speed);
                motor3.setPower(speed);
                motor4.setPower(speed);

            }else  if (robotDirection == "circle left"){
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(speed);
                motor4.setPower(speed);

            }else  if (robotDirection == "circle right"){
                motor1.setPower(-speed);
                motor2.setPower(-speed);
                motor3.setPower(-speed);
                motor4.setPower(-speed);

            }else{
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
            }

            telemetry.addData("motor1", motor1.getPower());
            telemetry.addData("motor2", motor2.getPower());
            telemetry.addData("motor3", motor3.getPower());
            telemetry.addData("motor4", motor4.getPower());

        }  catch (Exception p_exception) {
            //   telemetry.addData("98", "drive error" + p_exception.toString());
        }

    }
    void gyroDrive (String gyroDirection, double speed, double distance) {
        try {
            telemetry.addData("direction", gyroDirection);

            DcMotor flMotor = null;
            DcMotor frMotor = null;
            DcMotor blMotor = null;
            DcMotor brMotor = null;

            if (gyroDirection == "up") {
                flMotor = motor1;
                frMotor = motor2;
                brMotor = motor3;
                blMotor = motor4;


            } else if (gyroDirection == "right") {
                flMotor = motor2;
                frMotor = motor3;
                brMotor = motor4;
                blMotor = motor1;


            } else if (gyroDirection == "down") {
                flMotor = motor3;
                frMotor = motor4;
                brMotor = motor1;
                blMotor = motor2;


            } else if (gyroDirection == "left") {
                flMotor = motor4;
                frMotor = motor1;
                brMotor = motor2;
                blMotor = motor3;
            }





                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    flMotor.setDirection(DcMotor.Direction.FORWARD);
                    blMotor.setDirection(DcMotor.Direction.FORWARD);
                    frMotor.setDirection(DcMotor.Direction.REVERSE);
                    brMotor.setDirection(DcMotor.Direction.REVERSE);

                    flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                    // Determine new target position, and pass to motor controller
                    moveCounts = (int) (distance * COUNTS_PER_INCH);
                    newLeftTarget = flMotor.getCurrentPosition() + moveCounts;
                    newRightTarget = frMotor.getCurrentPosition() + moveCounts;

                    // Set Target and Turn On RUN_TO_POSITION
                    flMotor.setTargetPosition(newLeftTarget);
                    frMotor.setTargetPosition(newRightTarget);




                    // start motion.
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);

                    flMotor.setPower(speed/2);
                    frMotor.setPower(speed/2);
                    brMotor.setPower(speed/2);
                    blMotor.setPower(speed/2);


                    // keep looping while we are still active, and BOTH motors are running.
                    while (opModeIsActive() &&
                            flMotor.isBusy() && frMotor.isBusy()) {

                        // adjust relative speed based on headine3sg error.
                        error = getError(0);
                        steer = getSteer(error, P_DRIVE_COEFF);

                        // if driving in reverse, the motor correction also needs to be reversed
                        if (distance < 0)
                            steer *= -1.0;

                        leftSpeed = speed - steer;
                        rightSpeed = speed + steer;

                        // Normalize speeds if any one exceeds +/- 1.0;

                        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                        if (max > 1) {
                            leftSpeed /= max;
                            rightSpeed /= max;
                        }

                        flMotor.setPower(leftSpeed / 2);
                        blMotor.setPower(leftSpeed / 2);

                        frMotor.setPower(rightSpeed / 2);
                        brMotor.setPower(rightSpeed / 2);

                        // Display drive status for the driver.
                        telemetry.addData("direction", gyroDirection);
                        telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                        telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                        telemetry.addData("Actual", "%7d:%7d", flMotor.getCurrentPosition(),
                         frMotor.getCurrentPosition());
                        telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                        telemetry.addData("motor1", motor1.getPower());
                        telemetry.addData("motor2", motor2.getPower());
                        telemetry.addData("motor3", motor3.getPower());
                        telemetry.addData("motor4", motor4.getPower());

                        telemetry.addData("motor1 D", motor1.getDirection());
                        telemetry.addData("motor2 D", motor2.getDirection());
                        telemetry.addData("motor3 D", motor3.getDirection());
                        telemetry.addData("motor4 D", motor4.getDirection());
                        telemetry.update();

                        // Allow time for other processes to run.
                        idle();
                    }
                    // Stop all motion;
                    flMotor.setPower(0);
                    frMotor.setPower(0);

                    brMotor.setPower(0);
                    blMotor.setPower(0);

                    // Turn off RUN_TO_POSITION
                    flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
        }
            catch (Exception p_exception){

            }

        }

    }



