package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class drivercontrol extends OpMode {
    // constants for how far the arm can extend and retract
    private final int ARM_EXTEND_LIMIT;
    private final int ARM_RETRACT_LIMIT;

    // constants for how far the arm can rotate up and down
    private final int ARM_ROTATE_MAX;
    private final int ARM_ROTATE_MIN;

    // constant for the speed that the arm rotates with
    private final double ARM_ROTATIONAL_VELOCITY = 0.5;

    // constant for the speed that the arm extends and retracts with
    private final double ARM_EXTEND_SPEED = 0.5;

    // constants for the open and closed positions of the claw
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSE_POSITION = 0.1;

    // the DC motors for the wheels
    private final DcMotor leftWheelMotor, rightWheelMotor;

    // the DC motors for the arm
    private final DcMotor armRotationMotor, armExtensionMotor;

    // the servo motors for the pincers of the claw
    private final Servo pincerServo;

    // the servo that rotates the claw back and forth
    private final Servo clawRotationServo;

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
     * according to left joystick
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
     * Controls arm movement of the robot,
     * including both rotation and extension
     */
    public void moveArm() {
        this.extendArm();
        this.rotateArm();
    }

    /**
     * extends the arm back and forth
     */ 
    public void extendArm() {
        // get how far the arm is extended
        int armExtension = this.armExtensionMotor.getCurrentPosition();

        // if dpad_up is pressed and the arm is not extended
        if (gamepad1.dpad_up) {// && armExtension < this.ARM_EXTEND_LIMIT) {
            // set the target position to the max length of the arm
            this.armExtensionMotor.setTargetPosition(armExtension + 100);

            // move at a set speed 
            this.armExtensionMotor.setPower(this.ARM_EXTEND_SPEED);

            // extend the arm to its max length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad1.dpad_down) {// && armExtension > this.ARM_RETRACT_LIMIT) {
            // if dpad_down is pressed and the arm
            // is not fully retracted
            // set the target position to the min length of the arm
            this.armExtensionMotor.setTargetPosition(armExtension - 100);

            // move at a set speed
            this.armExtensionMotor.setPower(-this.ARM_EXTEND_SPEED);

            // retract the arm to its min length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Rotates the arm up and down
     */
    public void rotateArm() {
        double triggerVelocity = gamepad1.right_trigger - gamepad1.left_trigger;
        double bumperVelocity = ((gamepad1.right_bumper) ? 1 : 0) - ((gamepad1.left_bumper) ? 1 : 0);

        // how much the arm is rotated
        int armRotation = this.armRotationMotor.getCurrentPosition();

        // gradually raise or lower the arm 
        // if the right or left triggers are pressed, respectively
        if (Math.abs(triggerVelocity) > 0) {
            // get the sign of triggerVelocity
            int direction = (int) (Math.signum(triggerVelocity));
          
            // set the power of the motor
            this.armRotationMotor.setPower(direction * this.ARM_ROTATIONAL_VELOCITY);
        }

        // instantly raise or lower the arm 
        // if the right or left bumpers are pressed, respectively
        if (Math.abs(bumperVelocity) > 0) {
            // move the motor to a set position
            this.armRotationMotor.setTargetPosition(armRotation + 100 * bumperVelocity);
            this.armRotationMotor.setPower(bumperVelocity * this.ARM_ROTATIONAL_VELOCITY);
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
