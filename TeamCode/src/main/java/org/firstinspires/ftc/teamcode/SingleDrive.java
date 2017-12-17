package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Stephen Ogden on 11/19/2016.
 */

@TeleOp(name="Single TankDrive", group= "Linear Opmode")
@Disabled

public class SingleDrive extends LinearOpMode {

    DcMotor leftMotor = hardwareMap.dcMotor.get("Left motor");
    DcMotor rightMotor = hardwareMap.dcMotor.get("Right motor");
    DcMotor shooterMotor = hardwareMap.dcMotor.get("Shooter motor");
    DcMotor intakeMotor = hardwareMap.dcMotor.get("Intake motor");
    Servo servo1 = hardwareMap.servo.get("Servo 1");
    //Shooting position = 1.0
    //Open Position = 0.765
    //Closed Position = 0.617

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initalized");
        telemetry.addData("Config", "\"Left motor\", \"Right motor\", \"Shooter motor\", \"Intake motor\", and \"Servo 1\"");
        telemetry.update();

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // v
        rightMotor.setDirection(DcMotor.Direction.REVERSE); //AndyMark motors are retarded :|
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart(); //Press play dumbass

        shooterMotor.setTargetPosition(0);

        while (opModeIsActive()) {

            if(shooterMotor.isBusy() || gamepad1.a) {
                servo1.setPosition(1);
                if(shooterMotor.getTargetPosition() == 0 && gamepad1.a) {
                    shooterMotor.setTargetPosition(1600);
                }
            }
            if (gamepad1.right_bumper) {
                servo1.setPosition(0.765);
            } else {
                servo1.setPosition(0.617);
            }
            if (shooterMotor.getCurrentPosition() < 1650 && shooterMotor.getCurrentPosition() > 1550 && shooterMotor.getTargetPosition()==1600) {
                shooterMotor.setTargetPosition(0);
            }

            leftMotor.setPower(0.4*gamepad1.left_stick_y);
            rightMotor.setPower(0.4*gamepad1.right_stick_y);

            if (gamepad1.right_bumper) {
                intakeMotor.setPower(1.0);

            } else if (gamepad1.left_bumper) {
                    intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            shooterMotor.setPower(1.0);
            telemetry.addData("Target pos", shooterMotor.getTargetPosition());
            telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_y);
            telemetry.addData("Pos", shooterMotor.getCurrentPosition());
            telemetry.update();

        }

    }
}
