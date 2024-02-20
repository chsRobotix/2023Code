package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Near Autonomous")
public class redNearAutonomous extends LinearOpMode {
    private Hardware hardware = new Hardware(this);

    @Override
    public void runOpMode() {
        hardware.init();

        waitForStart();

        hardware.armRotationMotor.setTargetPosition(360);
        hardware.armRotationMotor.setPower(0.4);
        hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftWheelMotor.setTargetPosition(1200);
        hardware.leftWheelMotor.setPower(0.4);
        hardware.leftWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.rightWheelMotor.setTargetPosition(-1200);
        hardware.rightWheelMotor.setPower(0.4);
        hardware.rightWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(5000);
    }

    /**
     * Inititialize the variables for Tfod
     */
    public void initTfod() {
        // create a TensorFlow processor
        // robot.tfod = new TfodProcessor.Builder().build();

        // create a VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();
    }
}