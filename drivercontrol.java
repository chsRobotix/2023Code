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
     * Moves robot forward, backard, left, and right
     * according to left joystick of the gamepad1 
     */
    public void movement() {
        double turn = gamepad1.left_stick_x;
        double drive = gamepad1.left_stick_y;

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
        this.presetArmRotationPositions();
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
        int motorDirection = ((gamepad1.dpad_up) ? 1 : 0) - ((gamepad1.dpad_down) ? 1 : 0);

        if (motorDirection != 0
                && armExtension < this.ARM_EXTEND_LIMIT
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
        // gets the bumper direction from right and left bumperes
        // 1 for raising arm from min rotation; -1 for lowering arm to min rotation
        int bumperDirection = ((gamepad1.right_bumper) ? 1 : 0) - ((gamepad1.left_bumper) ? 1 : 0);

        // does not calculate further if bumpers are not pressed
        // or if both bumpers are pressed simultaneously
        if (bumperDirection == 0) {
            return;
        }

        // current rotational position of arm
        int position = armRotationMotor.getCurrentPosition();

        if (gamepad1.left_bumper && position > this.ARM_ROTATE_MIN) {
            if (position - 199 < this.ARM_ROTATE_MIN) {
                this.armRotationMotor.setTargetPosition(this.ARM_ROTATE_MIN);

            } else {
                this.armRotationMotor.setTargetPosition(position - 100);
            }

            this.armRotationMotor.setPower(-0.5);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad1.right_bumper && position < this.ARM_ROTATE_MAX) {
            if (position + 199 > this.ARM_ROTATE_MAX) {
                this.armRotationMotor.setTargetPosition(this.ARM_ROTATE_MAX);

            } else {
                this.armRotationMotor.setTargetPosition(position + 100);
            }

            this.armRotationMotor.setPower(0.5);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        return;

        // determines the rotational location of the arm after movement
        int targetPosition = position + this.ARM_ROTATIONAL_VELOCITY * bumperDirection;

        // checks if the target position overshoots arm limiters
        // prevents targeting beyond limiters
        if (targetPosition < this.ARM_ROTATE_MIN)
            targetPosition = this.ARM_ROTATE_MIN;

        if (targetPosition > this.ARM_ROTATE_MAX)
            targetPosition = this.ARM_ROTATE_MAX;

        // set the target position to the of the arm
        this.armRotationMotor.setTargetPosition(targetPosition);

        // move at a set speed
        this.armRotationMotor.setPower(0.5 * bumperDirection);

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
        // if the B button is pressed, open the claw
        if (gamepad1.b) {
            this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        } else if (gamepad1.x) { // if the X button is pressed, close the claw
            this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);
        }

        // if the Y button is pressed
        if (gamepad1.y) {
            // rotate the claw upward
            this.clawRotationServo.setPosition(0.0);

        } else if (gamepad1.a) { // if the A button is pressed
            // rotate the claw downward
            this.clawRotationServo.setPosition(1.0);
        }
    }
}
