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

        pushPixel();
        driveToBoard();

        dropPixel();
        sleep(4000);
        returnArmToInitialPosition();

        sleep(4000);
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
        hardware.rotateArm(50);
//        sleep(2000);

        // drive forward 29 inches
        hardware.drive(29);
//        sleep(5000);

        hardware.drive(-10);
//        sleep(2000);
    }

    /**
     * Drive to the scoring board
     */
    public void driveToBoard() {
        hardware.turn(-90);
//        sleep(1500);

        hardware.drive(-34);
    }

    /** 
     * Rotates the claw backward and drops the pixel
     */
    public void dropPixel() {
        telemetry.addData("armExtensionMotor position: ", hardware.armExtensionMotor.getCurrentPosition());
        hardware.armExtensionMotor.setTargetPosition(1300);
        hardware.armExtensionMotor.setPower(0.4);
        hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(3000);

        hardware.rotateArm(220);

//        sleep(2000);

        hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATE_MAX);
        sleep(1000);
        hardware.pincerServo.setPosition(hardware.CLAW_OPEN_POSITION);
    }

    /**
     * Moves the arm back to initial position
     */
    public void returnArmToInitialPosition() {
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
    }
}