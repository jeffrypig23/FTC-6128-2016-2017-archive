package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 12/10/2016.
 *
 * Shoots ball when the A button on gamepad 1 or 2 has been pressed
 */

@TeleOp(name="Shooter test", group="Test")  // @Autonomous(...) is the other common choice
@Disabled
public class shootertest extends LinearOpMode {

    DcMotor shooterMotor = null;

    byte shootingStage = 0;
    int lowestPos = -100;
    int startingPos = 0;
    int highestPos = 800;
    int runToPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Configured motors", "Shooter motor");

        shooterMotor = hardwareMap.dcMotor.get("Shooter motor");
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //No breaking at 0!

        waitForStart();

        while (opModeIsActive()) {

            //<editor-fold desc="Telemetry">
            if (Math.abs(shooterMotor.getCurrentPosition()) >= startingPos && shootingStage != 0) {
                telemetry.addData("Status", "Shooting...");
                telemetry.addData("Shooting Stage", shootingStage);
                telemetry.addData("Target Pos", shooterMotor.getTargetPosition());
                telemetry.addData("Current Pos", shooterMotor.getCurrentPosition());
                telemetry.update();
            }
            //</editor-fold>

            if ((gamepad2.a || gamepad1.a) && Math.abs(startingPos - shooterMotor.getCurrentPosition()) <= 10 && shootingStage == 0) {
                runToPos = highestPos;
                shootingStage = 3;
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
        }
    }
}