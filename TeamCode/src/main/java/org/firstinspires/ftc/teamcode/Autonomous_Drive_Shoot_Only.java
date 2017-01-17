package org.firstinspires.ftc.teamcode;

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
 * A lower scoring autonomous created in case another team has a similar main autonomous.
 * Scores 15 points.
 * 5 points for each particle scored-up to 2 that can be stored
 * 5 points for parking partially on the base.
 */


/*
    This would only drive and shoot - no beacon
    if other team is doing beacon - need this

 */
public class Autonomous_Drive_Shoot_Only extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void runDriveShoot(){
        try {
            init(hardwareMap);
            //Pulls ball up with ball elevator and then pitches with a 7 second timeout.
            telemetry.addData("Status", "Starting");
            telemetry.update();
            initSystem();
            runtime.reset();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            //Within 7 seconds, the 'ball elevator' will feed the balls into the pitching machine to launch them into the Center Vortex
            while (opModeIsActive()){

                if (teamColor == "blue"){
                    gyroDrive("left", DRIVE_SPEED, 7.5);
                } else {
                    gyroDrive("right", DRIVE_SPEED, 7.5);
                    encoderDrive(DRIVE_SPEED, "circle right", 3, 11);
                }
                //Tentative
                runtime.reset();
                while (runtime.seconds() > 6) {
                    beMotor.setPower(-100);
                    pMotor1.setPower(SPIN_SPEED);
                    pMotor2.setPower(-SPIN_SPEED);
                }
                if (teamColor == "blue"){

                    gyroDrive("left", DRIVE_SPEED, 15.5);
                } else {
                    gyroDrive ("right", DRIVE_SPEED, 15.5);
                }


            }

        } catch (Exception e) {
            telemetry.addData("runOpMode ERROR", e.toString());
            telemetry.update();
        }
    }

                // need to detect color, move to correct button, bump button
                // then back up and knock ball off stand and drive on stand



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



    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}





