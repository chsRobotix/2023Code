package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Red Near Autonomous")
public class redNearAutonomous extends LinearOpMode {
    private Hardware robot = new Hardware(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        drive(5);
        sleep(2000);
        drive(0);
    }

    /**
     * Inititialize the variables for Tfod
     */
    public void initTfod() {
        // create a TensorFlow processor
//        robot.tfod = new TfodProcessor.Builder().build();

        // create a VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();
    }

    /**
     * Set the power of the wheels to the same value
     */
    public void drive(double wheelPower) {
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