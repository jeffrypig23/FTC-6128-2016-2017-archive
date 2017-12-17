package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.PeerApp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 12/6/2016.
 *
 *Tankdrive
 *telemetry.addData("Right Stick X", gamepad1.right_stick_x);
 *telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
 */


@TeleOp(name="Frankdrive", group="Official")  // @Autonomous(...) is the other common choice
//@Disabled

public class FrankDrive extends LinearOpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor intakeMotor = null;
    DcMotor shooterMotor = null;

    Servo beaconRight = null;
    Servo beaconLeft = null;

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

    byte shootingStage = 0;

    int lowestPos = -100;
    int startingPos = 0;
    int highestPos = 800;
    int runToPos = 0;

    String Err = "None so far...";

    boolean turnButton;

    private ElapsedTime runtime = new ElapsedTime(2);

    enum DriveMode {TANK, SQUARED, WEST_COAST, ARCADE}

    @Override

    public void runOpMode() throws InterruptedException {

        leftMotor  = hardwareMap.dcMotor.get("Left motor");
        rightMotor = hardwareMap.dcMotor.get("Right motor");
        intakeMotor = hardwareMap.dcMotor.get("Intake motor");
        shooterMotor = hardwareMap.dcMotor.get("Shooter motor");

        beaconLeft = hardwareMap.servo.get("Beacon left");
        beaconRight = hardwareMap.servo.get("Beacon right");

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

        DriveMode driveMode = DriveMode.ARCADE; //TANK SQUARED WEST_COAST ARCADE
        //</editor-fold>


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Waiting", "Please press the play button!");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

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
                driveMode = DriveMode.TANK;
            }
            else if (gamepad1.b) { //square root
                driveMode = DriveMode.SQUARED;
            }
            else if (gamepad1.x) { //arcade
                driveMode = DriveMode.WEST_COAST;
            }
            else if (gamepad1.y) {
                driveMode = DriveMode.ARCADE;
            }
            //</editor-fold>

            //<editor-fold desc="Intake">
            if (gamepad2.right_bumper) {
                intakeMotor.setPower(-1.0);
            }
            else if(gamepad2.left_bumper) {
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
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
                //beaconrightpos = 0.85; //-- Old positions!
                //beaconleftpos = 0.05;
                double beaconrightpos = 0.67;
                double beaconleftpos = 0.33;
            }
            //</editor-fold>

            //<editor-fold desc="Shooter">
            if ((gamepad2.x) && Math.abs(startingPos - shooterMotor.getCurrentPosition()) <= 10 && shootingStage == 0) {
                runToPos = highestPos;
                shootingStage = 3;
            }
            if (Math.abs(shooterMotor.getCurrentPosition()) >= startingPos && shootingStage != 0) {
                telemetry.addData("Status", "Shooting...");
                telemetry.addData("Shooting Stage", shootingStage);
                telemetry.addData("Target Pos", shooterMotor.getTargetPosition());
                telemetry.addData("Current Pos", shooterMotor.getCurrentPosition());
                telemetry.update();
            }
            if (Math.abs(highestPos - shooterMotor.getCurrentPosition()) <= 10 && shootingStage == 3) {
                runToPos = lowestPos;
                shootingStage = 2;
            }
            if (Math.abs(lowestPos - shooterMotor.getCurrentPosition()) <= 10 && shootingStage == 2) {
                runToPos = startingPos;
                shootingStage = 1;
            }
            if (Math.abs(startingPos - shooterMotor.getCurrentPosition()) <= 10 && shootingStage == 1) {
                shootingStage = 0;
                telemetry.addData("Status", "Done!");
                telemetry.addData("Target Pos", shooterMotor.getTargetPosition());
                telemetry.addData("Current Pos", shooterMotor.getCurrentPosition());
                telemetry.update();
            }
            if (shootingStage != 0) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shooterMotor.setTargetPosition(runToPos);
                shooterMotor.setPower(1.0);
            } else {
                shooterMotor.setPower(0.0);
            }
            if ((gamepad2.x) && Math.abs(startingPos - shooterMotor.getCurrentPosition()) > 10 && shootingStage == 0) {
                Err = "Error resetting shooter to 0! Did someone move it?";
            }
            //</editor-fold>

            //<editor-fold desc="Telemetry">
            telemetry.addData("PowR", powR);
            telemetry.addData("PowL", powL);
            //telemetry.addData("PowR: " + powR, "PowL" + powL);
            telemetry.addData("Beacon Left Pos", beaconLeft.getPosition());
            telemetry.addData("Beacon Right Pos", beaconRight.getPosition());
            telemetry.addData("Left Motor Pos", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Pos", rightMotor.getCurrentPosition());
            telemetry.addData("Motor Mode", driveMode);
            telemetry.addData("Shooter errors", Err);
            telemetry.update();
            //</editor-fold>

        }
    }
    /*    ()()    */
    /*    ('.')   */
    /*    (()()   */
    /*   *(_()()  */
    /*   I am a   */
    /*Indentation */
    /*   bunny!   */

}