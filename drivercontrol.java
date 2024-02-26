package org.firstinspires.ftc.teamcode;

import javax.print.attribute.HashPrintRequestAttributeSet;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test")
public class drivercontrol extends OpMode {
    private Hardware hardware;

    // constant for the sensitivity of turning
    public final double TURNING_SENSITIVITY = 0.5;

    @Override
    public void init() {
        hardware = new Hardware(this);
        hardware.init();
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console

        driveWheels();
        moveArm();
        grabber();
        grabPixel();
        airplaneLauncher();

        telemetry.addData("Arm rotation position: ", hardware.armRotationMotor.getCurrentPosition());
        telemetry.addData("Arm extension position: ", hardware.armExtensionMotor.getCurrentPosition());
        telemetry.addData("Claw rotation position: ", hardware.clawRotationServo.getPosition());
        telemetry.addData("Airplane launcher position: ", hardware.airplaneLauncherServo.getPosition());
        //telemetry.addData("Potentiometer voltage: ", potentiometer.getVoltage());
    }

    /**
     * Controls wheel movement of the robot
     * Moves robot forward and backward according to left joystick of the gamepad1
     * Turns robot left and right according to right joystick of the gamepad1
     */
    public void driveWheels() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x * TURNING_SENSITIVITY;

        // power levels
        // motor gear rotation is inverse
        double leftWheelPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightWheelPower = Range.clip(drive - turn, -1.0, 1.0);

        hardware.leftWheelMotor.setPower(leftWheelPower);
        hardware.rightWheelMotor.setPower(rightWheelPower);
    }

    /**
     * Controls arm movement of the robot, including both rotation and extension
     */
    public void moveArm() {
        rotateArm();
        extendArm();
    }

    /**
     * Rotates arm outward and inward
     */
    public void rotateArm() {
        // get the current position of the arm
        int position = hardware.armRotationMotor.getCurrentPosition();

        if (gamepad2.right_stick_y > 0 && position > hardware.ARM_ROTATE_MIN) {
            /*
             * If rotating by ARM_ROTATE_SPEED would make the arm exceed the min
             *  rotate to ARM_ROTATE_MIN instead
             * Else
             *  rotate by ARM_ROTATE_SPEED
             */
            hardware.armRotationMotor.setTargetPosition(
                    Math.min(position - hardware.ARM_ROTATE_SPEED, hardware.ARM_ROTATE_MIN)
            );

            hardware.armRotationMotor.setPower(-0.15);
            hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(false);

        } else if (gamepad2.right_stick_y < 0 && position < hardware.ARM_ROTATE_MAX) {
            /*
             * If rotating by ARM_ROTATE_SPEED would make the arm exceed the max
             *  rotate to max instead
             * Else
             *  rotate by ARM_ROTATE_SPEED
             */
            hardware.armRotationMotor.setTargetPosition(
                    Math.min(position + hardware.ARM_ROTATE_SPEED, hardware.ARM_ROTATE_MAX)
            );

            hardware.armRotationMotor.setPower(0.15);
            hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(true);
        }

        telemetry.addData("Current arm position: ", position);
    }

    /**
     * Extends how far the robot arm extends and retracts
     */
    public void extendArm() {
        // get current position of motor
        int position = hardware.armExtensionMotor.getCurrentPosition();

        // armExtensionMax.getState() returns true when it is not being pressed
        // this will only run if the limit switch for the max arm extension has not been touched
        // if dpad_up is pressed and the max switch has not been hit
        // extend the arm
        if (gamepad2.dpad_up && hardware.armRetractionSwitch.getState()) {
            hardware.armExtensionMotor.setTargetPosition(position + hardware.ARM_EXTEND_SPEED);
            hardware.armExtensionMotor.setPower(0.4);
            hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.dpad_down && hardware.armExtensionSwitch.getState()) {
            // if dpad_down is pressed and the min switch has not been hit
            // retract the arm
            hardware.armExtensionMotor.setTargetPosition(position - hardware.ARM_EXTEND_SPEED);
            hardware.armExtensionMotor.setPower(-0.4);
            hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * As the arm rotates outward, it also retracts inward and vice versa
     * To counterbalance, the arm extends or retracts accordingly to the rotation
     *
     * @param isRotatingOutward whether the arm is rotating outwards
     */
    public void extendArmInResponse(boolean isRotatingOutward) {
        int position = hardware.armExtensionMotor.getCurrentPosition();

        // if the arm is being rotated outward, 
        // extend the arm outward too
        if (isRotatingOutward) {
            hardware.armExtensionMotor.setTargetPosition(position + hardware.ARM_EXTEND_SPEED / 2);
            hardware.armExtensionMotor.setPower(0.1);

        } else { // if the arm is being rotated inward,
            // retract the arm inward too
            hardware.armExtensionMotor.setTargetPosition(position - hardware.ARM_EXTEND_SPEED / 2);
            hardware.armExtensionMotor.setPower(-0.1);
        }

        hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            hardware.pincerServo.setPosition(hardware.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) {
            // if the right bumper is pressed, close the claw
            hardware.pincerServo.setPosition(hardware.CLAW_CLOSE_POSITION);
        }

        // get the current position of the claw rotation servo
        double currentClawRotationPosition = this.hardware.clawRotationServo.getPosition();

        // if the left trigger is pressed
        if (gamepad2.left_trigger > 0) {
            // rotate the claw upward
            this.hardware.clawRotationServo.setPosition(currentClawRotationPosition - hardware.CLAW_ROTATE_SPEED);

        } else if (gamepad2.right_trigger > 0) {
            // if the right trigger is pressed
            // rotate the claw downward
            this.hardware.clawRotationServo.setPosition(currentClawRotationPosition + hardware.CLAW_ROTATE_SPEED);
        }

        presetGrabberRotationPositions();
    }

    /**
     * Allows the driver to move the claw to two set positions:
     * up and down
     */
    public void presetGrabberRotationPositions() {
        // if Y Button is pressed,
        // rotate the claw upward
        if (gamepad2.y) {
            hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATION_HIGHEST_POSITION);

        } else if (gamepad2.a) {
            // if A button is pressed,
            // rotate the claw downward
            hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATION_LOWEST_POSITION);
        }
    }

    /**
     * Moves the arm and grabber into position to pick up a pixel
     * However, it does not close the claw
     */
    public void grabPixel() {
        // if x is pressed go to pixel grabbing position
        if (gamepad2.x) {
            hardware.armExtensionMotor.setTargetPosition(0);
            hardware.armExtensionMotor.setPower(-0.8);
            hardware.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hardware.armRotationMotor.setTargetPosition(hardware.ARM_ROTATE_MIN);
            hardware.armRotationMotor.setPower(-0.2);
            hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // rotate the claw back to its initial position
            hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATION_LOWEST_POSITION);
        }
    }

    /**
     * Rotates the arm back and drops the pixel
     */
    public void dropPixel() {


        hardware.armRotationMotor.setTargetPosition(hardware.ARM_ROTATE_MAX);
        hardware.armRotationMotor.setPower(-0.2);
        hardware.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.clawRotationServo.setPosition(hardware.CLAW_ROTATE_MAX);
        hardware.pincerServo.setPosition(hardware.CLAW_OPEN_POSITION);
    }

    /**
     * Launches airplane at a fixed angle
     */
    public void airplaneLauncher() {
        // if Y button is pressed
        // move the hook backward to release the rubber band
        if (gamepad1.y) {
            hardware.airplaneLauncherServo.setPosition(hardware.AIRPLANE_FIRING_POSITION);
        }
    }    
}