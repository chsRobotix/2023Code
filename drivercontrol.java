package org.firstinspires.ftc.teamcode;

import java.io.Serial;
import java.lang.Math;
import java.rmi.server.ServerCloneException;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class drivercontrol extends OpMode {
    // constants for how far the arm can extend and retract
    private final int ARM_EXTEND_LIMIT;
    private final int ARM_RETRACT_LIMIT;

    // constant for the speed that the arm spins with
    private final int ARM_ROTATIONAL_VELOCITY = 0.5;

    // the DC motors for the wheels
    private DcMotor leftWheelMotor, rightWheelMotor;

    // the DC motors for the arm
    private DcMotor armRotationMotor, armExtensionMotor;

    // the servo motors for the pincers of the claw
    private Servo leftPincerServo, rightPincerServo;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    private double leftWheelPower, rightWheelPower, armPower;

    @Override
    public void init() {
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        this.armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotator");
        this.armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extender");

        this.leftPincerServo = hardwareMap.get(Servo.class, "left_pincer_servo");
        this.rightPincerServo = hardwareMap.get(Servo.class, "right_pincer_servo");

        // setting the direction of the motors
        // rightWheelMotor and armRotationMotor are forward by default
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // setting the two pincer servo positions to 1, which is upright, aka not
        // pressing the claw
        this.leftPincerServo.setPosition(1.0);
        this.rightPincerServo.setPosition(1.0);

        // set the servo position of the grabber rotator to 1.0
        this.clawRotationServo.setPosition(1.0);

        this.leftWheelPower = 0;
        this.rightWheelPower = 0;

        this.armPower = 0;
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
        this.leftWheelPower = Range.clip(drive + turn, -1.0, 1.0);
        this.rightWheelPower = Range.clip(drive - turn, -1.0, 1.0);

        this.leftWheelMotor.setPower(this.leftWheelPower);
        this.rightWheelMotor.setPower(this.rightWheelPower);
    }

    /*
     * Controls arm movement of the robot,
     * including both rotation and extension
     */
    public void moveArm() {
        extendArm();
        rotateArm();
    }

    // extends the arm back and forth
    public void extendArm() {
        // set the motor to the power
        this.armRotationMotor.setPower(this.armPower);
        // get how far the arm is extended
        int armExtension = this.armExtensionMotor.getCurrentPosition();

        // if dpad_up is pressed and the arm is not extended
        if (gamepad1.dpad_up) {// && armExtension < this.ARM_EXTEND_LIMIT) {
            // set the target position to the max length of the arm
            this.armExtensionMotor.setTargetPosition(this.ARM_EXTEND_LIMIT);

            // move at max speed
            this.armExtensionMotor.setPower(0.5);

            // extend the arm to its max length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad1.dpad_down) {// && armExtension > this.ARM_RETRACT_LIMIT) {
            // if dpad_down is pressed and the arm
            // is not fully retracted
            // set the target position to the min length of the arm
            this.armExtensionMotor.setTargetPosition(this.ARM_RETRACT_LIMIT);

            // move at max speed
            this.armExtensionMotor.setPower(-0.5);

            // retract the arm to its min length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /*
     * Rotates the arm up and down
     */
    public void rotateArm() {
        double triggerVelocity = gamepad1.right_trigger - gamepad1.left_trigger;
        double bumperVelocity = 0;
        if (gamepad1.right_bumper)
            bumperVelocity++;

        if (gamepad1.left_bumper)
            bumperVelocity--;

        // gradually raise the arm
        if (Math.abs(triggerVelocity) > 0) {
            this.armRotationMotor.setPower(0.5);//triggerVelocity * this.ARM_ROTATIONAL_VELOCITY);
        }

        // if the right bumper is pressed
        if (Math.abs(bumperVelocity) > 0) {
        }
    }

    // Moves the grabber back and forth
    // Also closes and opens the claw
    public void grabber() {
        // if the left bumper is pressed, release the claw
        if (gamepad1.b) {
            this.leftPincerServo.setPosition(1.0);
            this.rightPincerServo.setPosition(1.0);

        } else if (gamepad1.x) { // if the right bumper is pressed, close the claw to half
            this.leftPincerServo.setPosition(0.5);
            this.rightPincerServo.setPosition(0.5);
        }

        // if the y button is pressed
        if (gamepad1.y) {
            // move the claws to position 0.0
            this.clawRotationServo.setPosition(0.0);

        } else if (gamepad1.a) { // if the b button is pressed
            // move the claw to position 1.0
            this.clawRotationServo.setPosition(1.0);
        }
    }
}
