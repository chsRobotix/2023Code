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

        dropPixel();

        hardware.pincerServo.setPosition(hardware.CLAW_CLOSE_POSITION);

        // retract the arm
        hardware.armExtensionMotor.setTargetPosition(0);
        hardware.armExtensionMotor.setPower(-0.8);
        hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // lower the arm
        hardware.armRotationMotor.setTargetPosition(hardware.ARM_ROTATE_MIN);
        hardware.armRotationMotor.setPower(-0.2);
        hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // rotate the claw back to its initial position
        hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATE_MIN);

        // turn 90 degrees left
        // hardware.turn(-90);

        hardware.rotateArm(90);

        sleep(2000);

        // drive forward 24 inches
        hardware.drive(30);

        sleep(5000);

        hardware.drive(-30);

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

    /** 
     * Rotates the claw backward and drops the pixel
     */
    public void dropPixel() {
        // lift the arm up by to level
        hardware.rotateArm(50);

        sleep(1500);

        // lift the arm up by to level
        hardware.rotateArm(50);

        hardware.armExtensionMotor.setTargetPosition(600);
        hardware.armExtensionMotor.setPower(0.4);
        hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.sleep(3000);

        hardware.rotateArm(210);

        opMode.sleep(2000);

        hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATE_MAX);
        opMode.sleep(1000);
        hardware.pincerServo.setPosition(hardware.CLAW_OPEN_POSITION);

        opMode.sleep(4000);
    }
}