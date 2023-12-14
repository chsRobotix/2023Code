package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;

@TeleOp(name = "Test Control")
public class drivercontrol extends OpMode {
    private Robot robot;

    /* wheel movement */
    // constant for the sensitivity of turning
    private final double TURNING_SENSITIVITY = 0.5;

    @Override
    public void init() {
        // create a robot object to initialize the robot
        robot = new Robot();
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console

        movement();
        moveArm();
        grabber();
        airplaneLauncher();
    }

    /**
     * Controls wheel movement of the robot
     * Moves robot forward and backward according to left joystick of the gamepad1
     * Turns robot left and right according to right joystick of the gamepad1
     * 
     */
    public void movement() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x * TURNING_SENSITIVITY;

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
        // get curreent position of motor
        int position = robot.armExtensionMotor.getCurrentPosition();

        // armExtensionMax.getState() returns true when it is not being pressed
        // this will only run if the limit switch for the max arm extension has not been touched
        // if dpad_up is pressed and the max switch has not been hit
        // extend the arm
        if (gamepad2.dpad_up && robot.armRetractionSwitch.getState()) {
            robot.armExtensionMotor.setTargetPosition(position + robot.ARM_EXTEND_SPEED);
            robot.armExtensionMotor.setPower(0.4);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.dpad_down && robot.armExtensionSwitch.getState()) {
            // if dpad_up is pressed and the max switch has not been hit
            // retract the arm
            robot.armExtensionMotor.setTargetPosition(position - robot.ARM_EXTEND_SPEED);
            robot.armExtensionMotor.setPower(-0.4);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    /**
     * Rotates arm outward and inward
     */
    public void rotateArm() {
        // get the current position of the arm
        int position = robot.armRotationMotor.getCurrentPosition();

        if (gamepad2.right_stick_y > 0 && position > robot.ARM_ROTATE_MIN) {
            // if the right stick is pressed down and the arm has not reached its min
            // rotate the arm inward
            if (position - robot.ARM_ROTATE_SPEED < robot.ARM_ROTATE_MIN) {
                // prevent the arm from exceeding its min
                robot.armRotationMotor.setTargetPosition(robot.ARM_ROTATE_MIN);

            } else {
                // move the arm inward by ARM_ROTATE_SPEED
                robot.armRotationMotor.setTargetPosition(position - robot.ARM_ROTATE_SPEED);
            }

            robot.armRotationMotor.setPower(-0.15);
            robot.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(false);

        } else if (gamepad2.right_stick_y < 0 && position < robot.ARM_ROTATE_MAX) {
            // if the right stick is pressed up and the arm has reached its max
            // rotate the arm outward
            if (position + robot.ARM_ROTATE_SPEED > robot.ARM_ROTATE_MAX) {
                // prevent the arm from exceeding its max
                robot.armRotationMotor.setTargetPosition(robot.ARM_ROTATE_MAX);

            } else {
                // rotate the arm outward by ARM_ROTATE_SPEED
                robot.armRotationMotor.setTargetPosition(position + robot.ARM_ROTATE_SPEED);
            }

            robot.armRotationMotor.setPower(0.15);
            robot.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            extendArmInResponse(true);
        }
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
        presetGrabberRotationPositions();

        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            // if the left bumper is pressed, open the claw
            robot.pincerServo.setPosition(robot.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) {
            // if the right bumper is pressed, close the claw
            robot.pincerServo.setPosition(robot.CLAW_CLOSE_POSITION);
        }

        // get the current position of the claw rotation servo
        double currentClawPosition = this.clawRotationServo.getPosition();

        // if the left trigger is pressed
        if (gamepad2.left_trigger > 0) {
            // rotate the claw upward
            this.clawRotationServo.setPosition(currentClawPosition - robot.CLAW_ROTATE_SPEED);

        } else if (gamepad2.right_trigger > 0) {
            // if the right trigger is pressed
            // rotate the claw downward
            this.clawRotationServo.setPosition(currentClawPosition + robot.CLAW_ROTATE_SPEED);
        }
    }

    /**
     * Allows the driver to move the claw to two set positions:
     * up and down
     */
    public void presetGrabberRotationPositions() {
        // if Y Button is pressed,
        // rotate the claw upward
        if (gamepad2.y) {
            robot.clawRotationServo.setPosition(0.0);

        } else if(gamepad2.a) {
            // if A button is pressed,
            // rotate the claw downward
            robot.clawRotationServo.setPosition(1.0);
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

}
