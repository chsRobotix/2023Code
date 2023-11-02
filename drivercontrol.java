package org.firstinspires.ftc.teamcode;

import java.io.Serial;
import java.rmi.server.ServerCloneException;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/*
 * Git commit example
 * 
 */

@TeleOp(name = "Driver Control")
public class DriverControl extends OpMode {
    // constants for how far the arm can extend and retract
    private final int ARM_EXTEND_LIMIT;
    private final int ARM_RETRACT_LIMIT;
    private final int ARM_ROTATIONAL_VELOCITY = 1;

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

        // setting the two servo positions to 1, which is upright, aka not pressing the
        // claw
        this.leftPincerServo.setPosition(1.0);
        this.rightPincerServo.setPosition(1.0);

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

    /**
     * Controls arm movement of the robot
     * controls the angle that the arm is at
     * as well as whether the arm is extended or not
     */

    public void moveArm() {

        // set armPower
        this.armPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);

        // set the motor to the power
        this.armRotationMotor.setPower(this.armPower);

        // get how far the arm is extended
        int armExtension = this.armExtensionMotor.getCurrentPosition();

        // if dpad_up is pressed and the arm is not extended
        if (gamepad1.dpad_up && armExtension < this.ARM_EXTEND_LIMIT) {
            // set the target position to the max length of the arm
            this.armExtensionMotor.setTargetPosition(this.ARM_EXTEND_LIMIT);

            // move at max speed
            this.armExtensionMotor.setPower(1.0);

            // extend the arm to its max length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad1.dpad_down && armExtension > this.ARM_RETRACT_LIMIT) {
            // if dpad_down is pressed and the arm
            // is not fully retracted
            // set the target position to the min length of the arm
            this.armExtensionMotor.setTargetPosition(this.ARM_RETRACT_LIMIT);

            // move at max speed
            this.armExtensionMotor.setPower(-1.0);

            // retract the arm to its min length
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Flicks the grabber back and forth
     * Also closes and opens the claw
     */
    public void grabber() {
        // grabber clutch and release
        if (gamepad1.left_bumper) {
            // if the left bumper is pressed, release the claw
            this.leftPincerServo.setPosition(1.0);
            this.rightPincerServo.setPosition(1.0);

        } else if (gamepad1.right_bumper) {
            // if the right bumper is pressed, close the claw to half
            this.leftPincerServo.setPosition(0.5);
            this.rightPincerServo.setPosition(0.5);
        }

        // grabber rotation
        if (gamepad1.a) {

        }
    }

    /**
     * 
     */
    public void rotateArm() {
        // calculates rotational velocity from triggers and bumper
        double triggerVelocity = gamepad1.right_trigger - gamepad1.left_trigger;
        double bumperVelocity = 0;
        if (gamepad1.left_bumper) {
            bumperVelocity--;
        }

        if (gamepad1.right_bumper) {
            bumperVelocity++;
        }

        // rotational limits


        // priorities gradual arm movement from trigger
        if (Math.abs(triggerVelocity) > 0) {
            // gradual arm movement from trigger
            this.armRotationMotor.setPower(triggerVelocity / this.ARM_ROTATIONAL_VELOCITY);

        } else if (Math.abs(bumperVelocity) > 0) {
            // instantaneous arm movement from bumper
            this.armRotationMotor.setPower(triggerVelocity * this.ARM_ROTATIONAL_VELOCITY);

        }

    }
}
