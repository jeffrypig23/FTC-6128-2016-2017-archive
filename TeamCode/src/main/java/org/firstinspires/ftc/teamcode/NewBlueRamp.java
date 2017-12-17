package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 1/14/2017.
 */

//This should drive to the right ramp

@Autonomous(name="NEW Blue Ramp Only", group="Test")  // @TeleOp(...) is the other common choice
//@Disabled

public class NewBlueRamp extends LinearOpMode{
    //<editor-fold desc="Initialization">
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intakeMotor = null;

    Servo beaconRight = null;
    Servo beaconLeft = null;

    private ElapsedTime runtime = new ElapsedTime();

    double beaconrightpos = 0.67;
    double beaconleftpos = 0.33;

    int stage = 0;
    int rightdiscrepency;
    int leftdiscrepency;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //</editor-fold>

        //<editor-fold desc="Run code">
        while (opModeIsActive()) {

            if (stage <= 3) {

                beaconLeft.setPosition(beaconleftpos);
                beaconRight.setPosition(beaconrightpos);

                leftMotor.setPower(0.3);
                rightMotor.setPower(0.3);

                //leftMotor.setTargetPosition();

                rightdiscrepency = Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition());
                leftdiscrepency = Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition());

                if (stage == 0) {
                    //Goes forward 26 inches
                    driveDistance(leftMotor, 26 * Inches * mmPerInch);
                    driveDistance(rightMotor, 26 * Inches * mmPerInch);
                    stage = 1;
                }

                if ((rightdiscrepency <= 50 || leftdiscrepency <= 50) && stage == 1) {
                    //Turns 90 degrees
                    driveDistance(leftMotor, 1.7 * Feet * mmPerInch);
                    driveDistance(rightMotor, 0.0 * Feet * mmPerInch);
                    stage = 2;
                }

                if ((leftdiscrepency <= 50) && stage == 2) {
                    //Drives forward 2.3 feet onto ramp
                    driveDistance(leftMotor, 2.3 * Feet * mmPerInch);
                    driveDistance(rightMotor, 2.3 * Feet * mmPerInch);
                    stage = 3;
                }

                if ((rightdiscrepency <= 50 || leftdiscrepency <= 50) && stage == 3) {
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
                    stage = 4;
                }

                telemetry.addData("Status", "Driving...");
                telemetry.addData("Descrepency (R/L", rightdiscrepency + "/" + leftdiscrepency);
                telemetry.addData("Motor current location (R/L)", rightMotor.getCurrentPosition() + "/" + leftMotor.getCurrentPosition());
                telemetry.addData("Motor target position (R/L)", rightMotor.getTargetPosition() + "/" + leftMotor.getTargetPosition());
                telemetry.addData("Stage", stage);
                telemetry.update();

            } else {
                //Stop Driving!
                //End of auto period
                leftMotor.setPower(0.0); //BRAKE
                rightMotor.setPower(0.0); //BRAKE
                telemetry.addData("Status", "Done!");
                telemetry.update();
            }
        }
        //</editor-fold>
    }
    public void driveDistance(DcMotor motor, double distance) { //drives forward distance millimeters
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int pos = (int) Math.round((distance/(wheelDiameter*Math.PI))*tickPerRotation);
        motor.setTargetPosition(pos);
       // motor.setPower(power);
    }
}
