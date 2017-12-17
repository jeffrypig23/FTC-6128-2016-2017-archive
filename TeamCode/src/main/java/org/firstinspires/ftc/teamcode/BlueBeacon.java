package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by Stephen Ogden on 12/10/2016.
 */

//This should drive to the right ramp

@Autonomous(name="Blue Beacon Auto", group="Test")  // @TeleOp(...) is the other common choice
@Disabled

public class BlueBeacon extends LinearVisionOpMode {
    //<editor-fold desc="Initialization">
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intakeMotor = null;

    Servo beaconRight = null;
    Servo beaconLeft = null;

    private ElapsedTime runtime = new ElapsedTime();

    double beaconrightpos = 0.85;
    double beaconleftpos = 0.05;

    final double Feet = 12.0;
    final double Inches = 1.0;
    final double mmPerInch = 25.4;
    final double wheelDiameter = 4*mmPerInch;
    final double tickPerRotation = 7*80; //the motor itself has 7 pulses per rotation however there is a 60:1 gearbox
    //</editor-fold>
    @Override

    public void runOpMode() throws InterruptedException {

        //<editor-fold desc="Declare and wait">
        leftMotor = hardwareMap.dcMotor.get("Left motor");
        rightMotor = hardwareMap.dcMotor.get("Right motor");
        intakeMotor = hardwareMap.dcMotor.get("Intake motor");

        beaconLeft = hardwareMap.servo.get("Beacon left");
        beaconRight = hardwareMap.servo.get("Beacon right");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //</editor-fold>

        //<editor-fold desc="Run code">
        while (opModeIsActive()) {

            beaconLeft.setPosition(beaconleftpos);
            beaconRight.setPosition(beaconrightpos);

            //Goes forward 26 inches
            driveDistance(leftMotor, 0.35, 26 * Inches * mmPerInch);
            driveDistance(rightMotor, 0.35, 26 * Inches * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 100 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 100) {
                telemetry.addData("Status", "Driving forward 27 inches...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();
            }

            //Wait half a second
            runtime.reset();
            while (runtime.seconds() < 1) {
                telemetry.addData("Status", "Waiting...");
                telemetry.update();
            }

            //Turns 90 degrees
            driveDistance(leftMotor, 0.30, 1.6 * Feet * mmPerInch);
            driveDistance(rightMotor, 0.0, 0.0 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 100 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 100) {
                telemetry.addData("Mode", "Turning...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();

            }

            //Wait half a second
            runtime.reset();
            while (runtime.seconds() < 1) {
                telemetry.addData("Status", "Waiting...");
                telemetry.update();
            }

            //Drives forward 2.3 feet onto ramp
            driveDistance(leftMotor, 0.3, 2.3 * Feet * mmPerInch);
            driveDistance(rightMotor, 0.3, 2.3 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 100 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 100) {
                telemetry.addData("Status", "Driving up ramp...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();
            }

            //Run intake motor, and brake others for 4 seconds
            //runtime.reset(); resets the counter back to 0
            runtime.reset();
            while (runtime.seconds() < 4) {
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
                intakeMotor.setPower(0.4);
                telemetry.addData("Status", "Gaining points...");
                telemetry.update();
            }

            //Turn off intake motor
            intakeMotor.setPower(0.0);

            //Drive back for 2.3 feet
            driveDistance(leftMotor, 0.20, -2.3 * Feet * mmPerInch);
            driveDistance(rightMotor, 0.20, -2.3 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 50) {
                telemetry.addData("Status", "Driving off ramp...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();
            }

            //Turns 90 degrees
            driveDistance(leftMotor, 0.0, 0.0 * Feet * mmPerInch);
            driveDistance(rightMotor, 0.30, 1.6 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 50) {
                telemetry.addData("Mode", "Turning...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();

            }

            //Drives forward 30 inches
            driveDistance(leftMotor, 0.35, 30.0 * Inches * mmPerInch);
            driveDistance(rightMotor, 0.35, 30.0 * Inches * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 50) {
                telemetry.addData("Status", "Driving to wall");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();
            }

            //Turn 45 degrees, to line up parrallel to the wall
            driveDistance(rightMotor, 0.30, 0.8 * Feet * mmPerInch);
            driveDistance(leftMotor, 0.0, 0.0 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 50) {
                telemetry.addData("Mode", "Turning...");
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                telemetry.update();

            }
            /*
            //TODO: GET VISION WORKING
            // Search for beacons
            driveDistance(rightMotor, 0.1, 2.0 * Feet * mmPerInch);
            driveDistance(leftMotor, 0.1, 2.0 * Feet * mmPerInch);
            while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) > 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) > 50 && beacon.getAnalysis().getColorString() == "???, ???") {
                telemetry.addData("Mode", "Searching for beacon...");
                telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
                telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                //telemetry.addData()
                telemetry.update();

            }
             */

            //Stop Driving!
            //End of auto period
            leftMotor.setPower(0.0); //BRAKE
            rightMotor.setPower(0.0); //BRAKE
            telemetry.addData("Status", "Done!");
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.update();
            sleep(20000); //Stop for time remaining...

        }
        //</editor-fold>
    }
    public void driveDistance(DcMotor motor, double power, double distance) { //drives forward distance millimeters
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int pos = (int) Math.round((distance/(wheelDiameter*Math.PI))*tickPerRotation);
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }
}
