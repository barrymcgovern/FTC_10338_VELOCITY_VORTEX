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
 * Created by kids on 1/18/2017.
 */
// @Autonomous(name="Comp: Vuforia Only", group="Pushbot")
public class OLDAutonomous_Vuforia extends Competition_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
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

            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.FORWARD);
            motor3.setDirection(DcMotor.Direction.FORWARD);
            motor4.setDirection(DcMotor.Direction.FORWARD);
            // Set all motors to zero power
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);



            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            speed = DRIVE_SPEED;
            //run until sensor is less than 17 cm,  or 6 seconds

            while (rangeSensor.rawUltrasonic() > 25 && (runtime.seconds() < 10)) {
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
            while (rangeSensor.rawUltrasonic() > 14 && (runtime.seconds() < 15)) {

                // Ask the listener for the latest information on where the robot is
                latestLocation = listener.getUpdatedRobotLocation();

                // The listener will sometimes return null, so we check for that to prevent errors
                if(latestLocation != null)
                    lastKnownLocation = latestLocation;

                coordinates = lastKnownLocation.getTranslation().getData();

                robotX = coordinates[0];
                robotY = coordinates[1];

                telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                telemetry.addData("Robot X " ,robotX);
                telemetry.addData("Robot Y " ,robotY);
                telemetry.update();

                speed = .1;

                if (robotY > 550 ) {
                    speed = .1;
                    drive("right");
                }else if ( robotY < 510) {
                    speed = .1;
                    drive("left");
                }else {
                    speed = .2;
                    drive("down");
                }
                telemetry.addData("Range_Sensor", rangeSensor.rawUltrasonic());
                telemetry.addData("Tracking " + target.getName(), listener.isVisible());
                telemetry.addData("Robot X " ,robotX);
                telemetry.addData("Robot Y " ,robotY);
                telemetry.update();
            }
    }

} public void initSystem() {
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



    private void oldsetupVuforia()
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

}
