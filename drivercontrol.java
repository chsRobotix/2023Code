package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test")
public class drivercontrol extends OpMode {
    private Hardware robot;

    @Override
    public void init() {
        robot = new Hardware(this);
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console

        movement();
        moveArm();
        grabber();
        grabPixelPosition();
        airplaneLauncher();

        telemetry.addData("Limit switch", robot.pincerLimiter.getState());
        telemetry.addData("Arm rotation position: ", robot.armRotationMotor.getCurrentPosition());
        telemetry.addData("Arm extension position: ", robot.armExtensionMotor.getCurrentPosition());
        telemetry.addData("Claw rotation position: ", robot.clawRotationServo.getPosition());
        telemetry.addData("Airplane launcher position: ", robot.airplaneLauncherServo.getPosition());
        //telemetry.addData("Potentiometer voltage: ", potentiometer.getVoltage());
    }

    /**
     * Controls wheel movement of the robot
     * Moves robot forward and backward according to left joystick of the gamepad1
     * Turns robot left and right according to right joystick of the gamepad1
     */
    public void movement() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x * robot.TURNING_SENSITIVITY;

        // power levels
        // motor gear rotation is inverse
        double leftWheelPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightWheelPower = Range.clip(drive - turn, -1.0, 1.0);

        robot.leftWheelMotor.setPower(leftWheelPower);
        robot.rightWheelMotor.setPower(rightWheelPower);
    }

    /**
     * Controls arm movement of the robot, including both rotation and extension
     */
    public void moveArm() {
        rotateArm();
        extendArm();
    }

    public void extendArm() {
        // get current position of motor
        int position = robot.armExtensionMotor.getCurrentPosition();

        // armExtensionMax.getState() returns true when it is not being pressed
        // this will only run if the limit switch for the max arm extension has not been touched
        // if dpad_up is pressed and the max switch has not been hit
        // extend the arm
        if (gamepad2.dpad_up && robot.armRetractionSwitch.getState() && robot.pincerLimiter.getState()) {
            robot.armExtensionMotor.setTargetPosition(position + robot.ARM_EXTEND_SPEED);
            robot.armExtensionMotor.setPower(0.4);
            robot.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.dpad_down && robot.armExtensionSwitch.getState()) {
            // if dpad_down is pressed and the min switch has not been hit
            // retract the arm
            robot.armExtensionMotor.setTargetPosition(position - robot.ARM_EXTEND_SPEED);
            robot.armExtensionMotor.setPower(-0.4);
            robot.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Rotates arm outward and inward
     */
    public void rotateArm() {
        // get the current position of the arm
        int position = robot.armRotationMotor.getCurrentPosition();

        if (gamepad2.right_stick_y > 0 && robot.pincerLimiter.getState()) {
            /*
             * If rotating by ARM_ROTATE_SPEED would make the arm exceed the min
             *  rotate to ARM_ROTATE_MIN instead
             * Else
             *  rotate by ARM_ROTATE_SPEED
             */
            robot.armRotationMotor.setTargetPosition(
                    Math.min(position - robot.ARM_ROTATE_SPEED, robot.ARM_ROTATE_MIN)
            );

            robot.armRotationMotor.setPower(-0.15);
            robot.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(false);

        } else if (gamepad2.right_stick_y < 0) {
            /*
             * If rotating by ARM_ROTATE_SPEED would make the arm exceed the max
             *  rotate to max instead
             * Else
             *  rotate by ARM_ROTATE_SPEED
             */
            robot.armRotationMotor.setTargetPosition(
                    Math.min(position + robot.ARM_ROTATE_SPEED, robot.ARM_ROTATE_MAX)
            );

            robot.armRotationMotor.setPower(0.15);
            robot.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(true);
        }

        telemetry.addData("Current arm position: ", position);
    }

    /**
     * As the arm rotates outward, it also retracts inward and vice versa
     * To counterbalance, the arm extends or retracts accordingly to the rotation
     *
     * @param isRotatingOutward whether the arm is rotating outwards
     */
    public void extendArmInResponse(boolean isRotatingOutward) {
        int position = robot.armExtensionMotor.getCurrentPosition();

        // if the arm is being rotated outward, 
        // extend the arm outward too
        if (isRotatingOutward) {
            robot.armExtensionMotor.setTargetPosition(position + robot.ARM_EXTEND_SPEED / 2);
            robot.armExtensionMotor.setPower(0.1);

        } else { // if the arm is being rotated inward,
            // retract the arm inward too
            robot.armExtensionMotor.setTargetPosition(position - robot.ARM_EXTEND_SPEED / 2);
            robot.armExtensionMotor.setPower(-0.1);
        }

        robot.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            robot.pincerServo.setPosition(robot.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) {
            // if the right bumper is pressed, close the claw
            robot.pincerServo.setPosition(robot.CLAW_CLOSE_POSITION);
        }

        // get the current position of the claw rotation servo
        double currentClawRotationPosition = this.robot.clawRotationServo.getPosition();

        // if the left trigger is pressed
        if (gamepad2.left_trigger > 0) {
            // rotate the claw upward
            this.robot.clawRotationServo.setPosition(currentClawRotationPosition - robot.CLAW_ROTATE_SPEED);

        } else if (gamepad2.right_trigger > 0) {
            // if the right trigger is pressed
            // rotate the claw downward
            this.robot.clawRotationServo.setPosition(currentClawRotationPosition + robot.CLAW_ROTATE_SPEED);
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
            robot.clawRotationServo.setPosition(robot.CLAW_ROTATION_HIGHEST_POSITION);

        } else if (gamepad2.a) {
            // if A button is pressed,
            // rotate the claw downward
            robot.clawRotationServo.setPosition(robot.CLAW_ROTATION_LOWEST_POSITION);
        }
    }

    /**
     * Launches airplane at a fixed angle
     */
    public void airplaneLauncher() {
        // if Y button is pressed
        // move the hook backward to release the rubber band
        if (gamepad1.y) {
            robot.airplaneLauncherServo.setPosition(robot.AIRPLANE_FIRING_POSITION);
        }
    }

    public void grabPixelPosition() {
        // if x is pressed go to pixel grabbing position
        if (gamepad2.x) {
            robot.armExtensionMotor.setTargetPosition(0);
            robot.armExtensionMotor.setPower(-0.8);
            robot.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armRotationMotor.setTargetPosition(0);
            robot.armRotationMotor.setPower(-0.2);
            robot.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // rotate the claw back to its initial position
            robot.clawRotationServo.setPosition(robot.CLAW_ROTATION_LOWEST_POSITION);
        }
    }
}
