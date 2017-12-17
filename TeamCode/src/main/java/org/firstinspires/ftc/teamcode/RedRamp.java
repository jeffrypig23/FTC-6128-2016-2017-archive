package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 12/10/2016.
 */

//This should drive to the right ramp

@Autonomous(name="Red Ramp Only", group="Official")  // @TeleOp(...) is the other common choice
//@Disabled

public class RedRamp extends LinearOpMode{
    //<editor-fold desc="Initialization">
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intakeMotor = null;

    Servo beaconRight = null;
    Servo beaconLeft = null;

    private ElapsedTime runtime = new ElapsedTime();

    double beaconrightpos = 0.67;
    double beaconleftpos = 0.33;

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

            beaconLeft.setPosition(beaconleftpos);
            beaconRight.setPosition(beaconrightpos);

            //Goes forward 25 inches
            driveDistance(leftMotor, 0.35, 25 * Inches * mmPerInch);
            driveDistance(rightMotor, 0.35, 25 * Inches * mmPerInch);
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
            driveDistance(leftMotor, 0.0, 0.0 * Feet * mmPerInch);
            driveDistance(rightMotor, 0.30, 1.9 * Feet * mmPerInch);
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

            //Stop Driving!
            //End of auto period
            leftMotor.setPower(0.0); //BRAKE
            rightMotor.setPower(0.0); //BRAKE
            telemetry.addData("Status", "Done!");
            telemetry.update();

            sleep(30000); //Stop for time remaining...

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
