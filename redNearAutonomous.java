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

        // lift the arm up by to level
        hardware.rotateArm(50);

        sleep(1500);

        hardware.armExtensionMotor.setTargetPosition(800);
        hardware.armExtensionMotor.setPower(0.4);
        hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(4000);

        hardware.rotateArm(135);
        hardware.pincerServo.setPosition(hardware.CLAW_OPEN_POSITION);

        // turn 90 degrees left
        // hardware.turn(-90);

        // drive forward 24 inches
//        hardware.drive(30);

//        sleep(5000);

//        hardware.drive(-30);

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