package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Near Autonomous")
public class redNearAutonomous extends LinearOpMode {
    private Hardware hardware = new Hardware(this);

    /**
     * Automatically runs after pressing init()
     */
    @Override
    public void runOpMode() {
        hardware.init();

        // wait until the player press the start button
        waitForStart();
        
        hardware.rotateArm(90);
        // hardware.armRotationMotor.setTargetPosition(360);
        // hardware.armRotationMotor.setPower(0.4);
        // hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // turn 90 degrees left
        hardware.turn(-90);

        // drive forward 24 inches
        hardware.drive(24);

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