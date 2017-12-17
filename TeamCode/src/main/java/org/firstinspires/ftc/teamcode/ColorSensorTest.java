package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;

/**
 * Created by Stephen Ogden on 1/4/2017.
 */

@TeleOp(name = "Color sensors test", group = "Test")
@Disabled

// This uses both the hardware color sensors, and a vision opMode from a custom library,
// which utilizes the camera on the phone

public class ColorSensorTest extends LinearVisionOpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intakeMotor = null;
    DcMotor shooterMotor = null;

    Servo beaconRight = null;
    Servo beaconLeft = null;

    ColorSensor colorSensor;

    double turnSpeed = 0.8;
    double powR;
    double powL;
    double throttle;
    double turn;
    double beaconrightpos = 0.67;
    double beaconleftpos = 0.33;

    //final double mmPerInch = 25.4;
    //final double wheelDiameter = 4*mmPerInch;
    //final double tickPerRotation = 7*80; //the motor itself has 7 pulses per rotation however there is a 60:1 gearbox

    String Err = "None so far...";
    String returnedColor = "Can't tell :(";

    boolean turnButton;

    private ElapsedTime runtime = new ElapsedTime(2);

    //enum DriveMode {TANK, SQUARED, WEST_COAST, ARCADE}

    @Override

    public void runOpMode() throws InterruptedException{

        //<editor-fold desc="Initialization and Declaration">
        leftMotor  = hardwareMap.dcMotor.get("Left motor");
        rightMotor = hardwareMap.dcMotor.get("Right motor");
        intakeMotor = hardwareMap.dcMotor.get("Intake motor");
        shooterMotor = hardwareMap.dcMotor.get("Shooter motor");

        beaconLeft = hardwareMap.servo.get("Beacon left");
        beaconRight = hardwareMap.servo.get("Beacon right");

        colorSensor = hardwareMap.colorSensor.get("Color sensor");

        //<editor-fold desc="Motor behavior">
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //No breaking at 0!
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrankDrive.DriveMode driveMode = FrankDrive.DriveMode.ARCADE; //TANK SQUARED WEST_COAST ARCADE
        //</editor-fold>

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
        //</editor-fold>

        //<editor-fold desc="Vision Lib Declaration">
        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        //Wait for the match to begin
        //</editor-fold>

        waitForStart();

        while (opModeIsActive()) {

            rightMotor.setPower(powR);
            leftMotor.setPower(powL);

            beaconLeft.setPosition(beaconleftpos);
            beaconRight.setPosition(beaconrightpos);

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            turnButton = gamepad1.left_stick_button;
            //<editor-fold desc="Switch">
            switch(driveMode) {
                case TANK:
                    powL = -gamepad1.left_stick_y;
                    powR = -gamepad1.right_stick_y;
                    break;
                case SQUARED:
                    powR = -gamepad1.right_stick_y * (Math.abs(gamepad1.right_stick_y));
                    powL = -gamepad1.left_stick_y * (Math.abs(gamepad1.left_stick_y));
                    break;
                case WEST_COAST: //wont turn unless throttle
                    if (turn < 0) { //want to turn left
                        powR = (-throttle);
                        powL = (-throttle) * (1.0-turnSpeed*Math.abs(turn));
                    }
                    else { //turn right
                        powR = (-throttle) * (1.0-turnSpeed*Math.abs(turn));
                        powL = (-throttle);
                    }
                    if (gamepad1.left_stick_button) {
                        powR = -turn;
                        powL = turn;

                    }
                    break;
                case ARCADE: // will turn in place
                    throttle = throttle * Math.abs(throttle); // James Added --NO TOUCHING

                    powR = (-throttle) - (turn*(Math.abs(throttle)));
                    powL = (-throttle) + (turn*(Math.abs(throttle)));

                    if (Math.abs(powR) > 1)  {
                        //powL=(1.0/Math.abs(powR)) * powL;
                        powR=(1.0/Math.abs(powR)) * powR;
                    }
                    if(Math.abs(powL) > 1) {
                        //powR=(1.0/Math.abs(powL)) * powR;
                        powL=(1.0/Math.abs(powL)) * powL;
                    }
                    if(throttle==0) {
                        powR = -0.8 * turn;
                        powL = 0.8 * turn;
                    }
                    break;
            }

            if (gamepad1.a) { //arcade
                driveMode = FrankDrive.DriveMode.TANK;
            }
            else if (gamepad1.b) { //square root
                driveMode = FrankDrive.DriveMode.SQUARED;
            }
            else if (gamepad1.x) { //arcade
                driveMode = FrankDrive.DriveMode.WEST_COAST;
            }
            else if (gamepad1.y) {
                driveMode = FrankDrive.DriveMode.ARCADE;
            }
            //</editor-fold>

            //<editor-fold desc="Servos">
            if (gamepad2.dpad_left) {
                beaconleftpos = beaconleftpos + 0.02;
                beaconrightpos = beaconrightpos - 0.02;
            }
            if (gamepad2.dpad_right) {
                beaconleftpos = beaconleftpos - 0.02;
                beaconrightpos = beaconrightpos + 0.02;
            }
            if (beaconleftpos > 1.0) {
                beaconleftpos = 1.0;
            }
            if (beaconleftpos < 0.0) {
                beaconleftpos = 0.0;
            }
            if (beaconrightpos > 1.0) {
                beaconrightpos = 1.0;
            }
            if (beaconrightpos < 0.0) {
                beaconrightpos = 0.0;
            }

            if (gamepad2.y) { //RESET
                //beaconrightpos = 0.85;
                //beaconleftpos = 0.05;
                double beaconrightpos = 0.67;
                double beaconleftpos = 0.33;
            }
            //</editor-fold>

            //<editor-fold desc="Color sensor">
            if (gamepad1.x || gamepad2.x)  { // Toggle LED when pressing X
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            if (colorSensor.red() > colorSensor.blue()) {
                returnedColor = "Red";
            } else if (colorSensor.blue() > colorSensor.red()) {
                returnedColor = "Blue";
            } else {
                returnedColor = "Can't tell :(";
            }
            //</editor-fold>

            //<editor-fold desc="Telemetry">
            telemetry.addData("Custom Lib Beacon Color", beacon.getAnalysis().getColorString());
            //telemetry.addData("Custom Lib Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Custom Lib Frame Rate", fps.getFPSString() + " FPS");
            //telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Color Sensor Red  ", colorSensor.red());
            //telemetry.addData("Color Sensor Green", colorSensor.green());
            telemetry.addData("Color Sensor Blue ", colorSensor.blue());
            telemetry.addData("Color Sensor Hue", hsvValues[0]);
            telemetry.addData("Color sensor Color", returnedColor);
            //telemetry.addData("Status", "Done");

            telemetry.update();
            //</editor-fold>

            //sleep(30000);

            waitOneFullHardwareCycle();
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

        }
    }
}
