package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Team: Dark Matters #10338
 * Velocity Vortex
 * The main autonomous program that will claim the beacon for either red or blue
 * Claims a beacon for 30 points
 * Knocks off cap ball for 5 points
 * Partially parks on base for 5 points
 */


public class OLDAutonomous_Master extends Competition_Hardware {

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void runMaster(){
        try {
            init(hardwareMap);

            telemetry.addData("Status", "Starting");    //
            telemetry.update();
            speed = DRIVE_SPEED;
            telemetry.addData("Status", "init");    //
            telemetry.update();
            initSystem(); // See initSystem below
            telemetry.addData("Status", "wait");    //
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive()) {

                // Send telemetry message to show position;
                telemetry.addData("motor1", motor1.getCurrentPosition());
                telemetry.addData("motor2", motor2.getCurrentPosition());
                telemetry.addData("motor3", motor3.getCurrentPosition());
                telemetry.addData("motor4", motor4.getCurrentPosition());
                telemetry.update();

                // Has the robot go forward at a set speed, in a set direction, for 14 inches with a 5 second timeout
                if (teamColor == "blue"){

                    gyroDrive("left", DRIVE_SPEED, 15.5);
                } else {
                    gyroDrive ("right", DRIVE_SPEED, 15.5);
                }

                runtime.reset();

                // Motors will run without encoders in next step
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                //run until sensor is less than 17 cm,  or 6 seconds

                while (rangeSensor.rawUltrasonic() > 25 && (runtime.seconds() < 6)) {
                    telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                    telemetry.update();
                    drive("down");
                }

                drive("stop");

                // Ask the listener for the latest information on where the robot is
                latestLocation = listener.getUpdatedRobotLocation();

                // The listener will sometimes return null, so we check for that to prevent errors
                if(latestLocation != null)
                    lastKnownLocation = latestLocation;

                coordinates = lastKnownLocation.getTranslation().getData();

                robotX = coordinates[0];
                robotY = coordinates[1];
                robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                // Send information about whether the target is visible, and where the robot is
                telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                telemetry.addData("Robot X " ,robotX);
                telemetry.addData("Robot Y " ,robotY);
                telemetry.update();
                while (rangeSensor.rawUltrasonic() > 17 && (runtime.seconds() < 6)) {
                     if (robotY > 550 ) {
                      drive("right");
                  }else if ( robotY < 450) {
                         drive("left");
                  }else {
                         drive("down");
                   }
                }

                // needs to be inside loop
                // Uses the more specific range sensor to make sure that robot is close enough for color sensor before stopping

                // will move the robot a certain way and then move into the button
                // will need timeout and then move back to neutral afterwards
                //determines what the color is and what the alliance color is set to determine which way to move
                if (colorSensor.red() > colorSensor.blue()) {
                    telemetry.addData("1", "Red", colorSensor.red());
                    if (teamColor == "blue") {
                        encoderDrive(DRIVE_SPEED, "left", 1, 10);
                        encoderDrive(DRIVE_SPEED, "down", 1, 12);
                    } else {
                        encoderDrive(DRIVE_SPEED, "right", 1, 10);
                        encoderDrive(DRIVE_SPEED, "down", 1, 12);

                    }
                    //if the color detected is blue...
                }else if (colorSensor.blue() > colorSensor.red()){
                    telemetry.addData("1", "Blue", colorSensor.blue());
                    if (teamColor == "blue"){
                        encoderDrive(DRIVE_SPEED, "right", 1, 10);
                        encoderDrive(DRIVE_SPEED, "down", 1, 12);
                    } else {
                        encoderDrive(DRIVE_SPEED, "left", 1, 10);
                        encoderDrive(DRIVE_SPEED, "down", 1, 12);
                    }
                }
                //moves backwards
                encoderDrive(DRIVE_SPEED, "up", 16 ,14 );
                break;

            }

        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

    public void initSystem() {
        try {
            // At the beginning of autonomous program, the encoders on the motors are reset

            telemetry.addData("Status", "inside init");    //
            telemetry.update();
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "b4 gyro");    //
            telemetry.update();
            gyro.calibrate();

            while (gyro.isCalibrating())  {
                Thread.sleep(50);
                idle();
            }
            telemetry.addData("Status", "after gyro");    //
            telemetry.update();
            gyro.resetZAxisIntegrator();
            setupVuforia();

            // We don't know where the robot is, so set it to the origin
            // If we don't include this, it would be null, which would cause errors later on
            lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);


            visionTargets.activate();
            OpenGLMatrix latestLocation;
            float[] coordinates;






        } catch (Exception e) {

            telemetry.addData("initSystem ERROR", e.toString());
            telemetry.update();
        }

    }




}






