package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Autonomous")
public class autonomous extends LinearOpMode {
    private Robot robot;

    /* variables for the vision system */

    // Variable to store and instance of the TensorFlow Object Detection(TFOD)
    // processor.
    private TfodProcessor tfod;

    // variable to store an instane of VisionPortal
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        // drive forward and backwards to test
        setWheelPower(1.0);
        sleep(1000);
        setWheelPower(-1.0);

        telemetry.addData("", "Done running!");
    }

    /**
     * Inititialize the variables for Tfod
     */
    public void initTfod() {
        // create a TensorFlow processor
        tfod = new TfodProcessor.Builder().build();

        // create a VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();
    }

    /**
     * Set the power of the wheels to the same value
     */
    public void setWheelPower(double wheelPower) {
        robot.leftWheelMotor.setPower(wheelPower);
        robot.rightWheelMotor.setPower(wheelPower);
    }

    /**
     * Turns the robot a certain number of degrees
     * Negative is left
     * Positive is right
     */
    public void turn(double degrees) {
        double wheelPower = 1.0;
    }
}