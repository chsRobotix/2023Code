package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Far Autonomous")
public class blueFarAutonomous extends LinearOpMode {
    private Hardware hardware = new Hardware(this);

    /**
     * Automatically runs after pressing init()
     */
    @Override
    public void runOpMode() {
        hardware.init();

        hardware.rotateArm(90);
        hardware.rotateArm(0);
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

    /**
     * Push the pixel onto the tape
     */
    public void pushPixel() {
    }

    /**
     * Drive to the scoring board
     */
    public void driveToBoard() {
    }

    /**
     * Rotates the claw backward and drops the pixel
     */
    public void dropPixel() {
    }

    /**
     * Moves the arm back to initial position
     */
    public void returnArmToInitialPosition() {
    }
}