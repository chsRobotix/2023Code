package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class drivercontrol extends OpMode {
    // constants for how far the arm can extend and retract
    private final int ARM_EXTEND_LIMIT = 0;
    private final int ARM_RETRACT_LIMIT = 0;

    // constants for how far the arm can rotate up and down
    private final int ARM_ROTATE_MAX = 2000;
    private final int ARM_ROTATE_MIN = 0;
    private final int ARM_ROTATE_MID = (this.ARM_ROTATE_MAX + this.ARM_ROTATE_MIN) / 2;

    // constant for the speed that the arm rotates with
    private final double ARM_ROTATIONAL_VELOCITY = 100;

    // constant for the speed that the arm extends and retracts with
    private final double ARM_EXTEND_SPEED = 0.5;

    // constants for the open and closed positions of the claw
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSE_POSITION = 0.1;

    // the DC motors for the wheels
    private DcMotor leftWheelMotor, rightWheelMotor;

    // the DC motors for the arm
    private DcMotor armRotationMotor, armExtensionMotor;

    // the servo motors for the pincers of the claw
    private Servo pincerServo;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    @Override
    public void init() {
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        this.armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotator");
        this.armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extender");

        this.pincerServo = hardwareMap.get(Servo.class, "pincer_servo");

        // setting the direction of the motors
        // rightWheelMotor and armRotationMotor are forward by default
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // setting the two pincer servo positions to 1, which is upright, aka not
        // pressing the claw
        this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        // set the servo position of the grabber rotator to 1.0
        this.clawRotationServo.setPosition(1.0);
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console

        this.movement();
        this.moveArm();
        this.grabber();
    }

    /**
     * Controls wheel movement of the robot
     * Moves robot forward and backard according to left joystick of the gamepad1
     * Turns robot left and right according to right joystick of the gamepad1
     * 
     */
    public void movement() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // power levels
        // motor gear rotation is inversed
        double leftWheelPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightWheelPower = Range.clip(drive - turn, -1.0, 1.0);

        this.leftWheelMotor.setPower(leftWheelPower);
        this.rightWheelMotor.setPower(rightWheelPower);
    }

    /**
     * Controls arm movement of the robot, including both rotation and extension
     */
    public void moveArm() {
        this.extendArm();
        this.rotateArm();
    }

    /**
     * Extends the arm back and forth with the dpad on the gamepad2
     */
    public void extendArm() {
        // get how far the arm is extended
        int armExtension = this.armExtensionMotor.getCurrentPosition();

        // get the direction that the motor will rotate in
        // if only dpad_up is pressed, it moves forward
        // if only dpad_down is pressed, it moves backward
        int motorDirection = ((gamepad2.dpad_up) ? 1 : 0) - ((gamepad2.dpad_down) ? 1 : 0);

        if (motorDirection == 0) {
            return;
        }

        if (armExtension < this.ARM_EXTEND_LIMIT
                && armExtension > this.ARM_RETRACT_LIMIT) {
            // set the target position to the of the arm
            this.armExtensionMotor.setTargetPosition(armExtension + 100 * motorDirection);

            // move at a set speed
            this.armExtensionMotor.setPower(this.ARM_EXTEND_SPEED * motorDirection);

            // set the arm to move to position
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Rotates the arm up and down with bumpers on gamepad2
     */
    public void rotateArm() {
        this.presetArmRotationPositions();

        // gets the bumper direction from right joystick of gamepad 2
        int rotateDirection = (int) Math.signum(gamepad2.right_stick_x);

        // does not calculate further if bumpers are not pressed
        // or if both bumpers are pressed simultaneously
        if (rotateDirection == 0) {
            return;
        }

        // current rotational position of arm
        int position = armRotationMotor.getCurrentPosition();

        // determines the rotational location of the arm after movement
        int targetPosition = position + this.ARM_ROTATIONAL_VELOCITY * rotateDirection;

        // checks if the target position overshoots arm limiters
        // prevents targeting beyond limiters
        if (targetPosition < this.ARM_ROTATE_MIN)
            targetPosition = this.ARM_ROTATE_MIN;

        if (targetPosition > this.ARM_ROTATE_MAX)
            targetPosition = this.ARM_ROTATE_MAX;

        // set the target position to the of the arm
        this.armRotationMotor.setTargetPosition(targetPosition);

        // move at a set speed
        this.armRotationMotor.setPower(0.5 * rotateDirection);

        // set the arm to move to position
        this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Moves the arm to hard-coded positions of min, mid, and max
     * using buttons on gamepad2
     */
    public void presetArmRotationPositions() {
        if (gamepad2.a) {
            // moves arm to minimum rotation position
            this.armRotationMotor.setTargetPosition(this.ARM_ROTATE_MIN);
            this.armRotationMotor.setPower(-1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.b) {
            // determines which direction to move
            // if below mid rotation position, motorDirection = 1
            // if above mid rotation position, motorDirection = -1
            int motorDirection = Math.signum(this.ARM_ROTATE_MID - this.armRotationMotor.getCurrentPosition);

            // moves arm to mid rotation position
            this.armRotationMotor.setTargetPosition(this.ARM_ROTATE_MID);
            this.armRotationMotor.setPower(1.0 * motorDirection);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.y) {
            // moves arm to maximum rotation position
            this.armRotationMotor.setTargetPosition(this.ARM_ROTATE_MAX);
            this.armRotationMotor.setPower(1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        // gets the bumper direction from right and left bumpers of gamepad 2
        // 1 for closing grabber; -1 for opening grabber
        int bumperDirection = ((gamepad2.right_bumper) ? 1 : 0) - ((gamepad2.left_bumper) ? 1 : 0);

        // gets the bumper direction from right and left triggers of gamepad 2
        // 1 for rotating claw upward; 1 for rotating claw downward
        int triggerDirection = Math.signum(gamepad2.right_trigger - gamepad2.left_trigger);

        if (bumperDirection > 0) {
            // opens claw if right bumper is pressed
            this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        } else if (bumperDirection < 0) {
            // closes claw if left bumper is pressed
            this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);
        }

        if (triggerDirection > 0) {
            // rotate the claw upward
            this.clawRotationServo.setPosition(0.0);

        } else if (triggerDirection < 0) {
            // rotate the claw downward
            this.clawRotationServo.setPosition(1.0);
        }
    }
}
