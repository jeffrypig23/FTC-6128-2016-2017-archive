package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 12/8/2016.
 */

@Autonomous(name="Capball only", group="Official")  // @TeleOp(...) is the other common choice
//@Disabled

public class CapballAuto extends LinearOpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    final double mmPerInch = 25.4;
    final double wheelDiameter = 4*mmPerInch;
    final double tickPerRotation = 7*80; //the motor itself has 7 pulses per rotation however there is a 60:1 gearbox

    private ElapsedTime runtime = new ElapsedTime();
    //private ElapsedTime autotime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Configured motors", "Left motor, and Right motor");

        leftMotor = hardwareMap.dcMotor.get("Left motor");
        rightMotor = hardwareMap.dcMotor.get("Right motor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set the phone side as the front of the robot (if you set both motors to 1 it will drive in the direction of the phone)
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

                while (runtime.seconds() < 4) {
                    telemetry.addData("Status", "Waiting...");
                    telemetry.update();
                }
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                driveDistance(rightMotor, 0.4, 4.5 * 12 * mmPerInch); //and 4.5inches
                driveDistance(leftMotor, 0.4, 4.5 * 12 * mmPerInch); // 4.5 feet
                while (Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition()) < 50 || Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition()) < 50) {
                    telemetry.addData("Mode", "driving forward");
                    telemetry.addData("Target Pos", "R%d L%d", rightMotor.getTargetPosition(), leftMotor.getTargetPosition());
                    telemetry.addData("Current Pos", "R%d L%d", rightMotor.getCurrentPosition(), leftMotor.getCurrentPosition());
                    telemetry.update();
                }
                    telemetry.addData("Status", "Done!");
                    telemetry.update();
                runtime.reset();


            sleep(300000);

        }
        telemetry.addData("Status", "Done!");
        telemetry.update();

    }
    public void driveDistance(DcMotor motor ,double power, double distance) { //drives forward distance millimeters
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int pos = (int) Math.round((distance/(wheelDiameter*Math.PI))*tickPerRotation);
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }

}
