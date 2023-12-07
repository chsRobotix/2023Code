package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;

@TeleOp(name = "Driver Control")
public class drivercontrol extends OpMode {
    /* wheel movement */
    // constant for the sensitivity of turning
    private final double TURNING_SENSITIVITY = 0.5;

    // the DC motors for the wheels
    private DcMotor leftWheelMotor, rightWheelMotor;

    /* arm rotation */
    // constants for how far the arm can rotate outward and inward
    private final int ARM_ROTATE_MAX = 2000;
    private final int ARM_ROTATE_MIN = 0;
    private final int ARM_ROTATE_SPEED = 50;

    // the DC motors for the arm
    private DcMotor armRotationMotor;

    /* arm extension */
    // constant for the speed that the arm extends and retracts with
    private final int ARM_EXTEND_SPEED = 50;

    // DC motor for extending the arm
    private DcMotor armExtensionMotor;

    // the limit switches for arm extension and retraction
    private DigitalChannel armExtensionSwitch;
    private DigitalChannel armRetractionSwitch;

    /* claw */
    // constants for the open and closed positions of the claw
    private final double CLAW_OPEN_POSITION = 1.0;
    private final double CLAW_CLOSE_POSITION = 0.075;

    // constants for how fast the claw rotates
    private final double CLAW_ROTATE_SPEED = 0.003;

    // the servo motors for the pincers of the claw
    private Servo pincerServo;

    // the limit switches for the pincers of the claw
    private DigitalChannel clawOpenSwitch;
    private DigitalChannel clawCloseSwtich;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    /* airplane */
    // starting and ending position for airplane launcher
    private final double AIRPLANE_LOADED_POSITION = 1.0;
    private final double AIRPLANE_FIRING_POSITION = 0.5;

    // the servo that launches the airplane
    private Servo airplaneLauncherServo;

    @Override
    public void init() {
        /* wheel movement */
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // setting the direction of the motors
        // rightWheelMotor is forward by default
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* arm rotation */
        this.armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotator");
        this.armRotationMotor.resetDeviceConfigurationForOpMode();

        /* arm extension */
        this.armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extender");
        this.armExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // allows extension motor to coast while not in use
        // prevents arm from retracting during rotation
        this.armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.armRetractionSwitch = hardwareMap.get(DigitalChannel.class, "armExtensionMax");
        this.armExtensionSwitch = hardwareMap.get(DigitalChannel.class, "armExtensionMin");

        /* claw */
        // setting the two pincer servo positions to open
        this.pincerServo = hardwareMap.get(Servo.class, "pincer_servo");

        this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);

        // set the servo position of the grabber rotator to prevent ground collision
        this.clawRotationServo = hardwareMap.get(Servo.class, "pincer_rotation_servo");
        this.clawRotationServo.setPosition(0.0);

        /* airplane */
        // set the servo position of airplaneLauncherServo to stretch rubber band
        this.airplaneLauncherServo = hardwareMap.get(Servo.class, "airplane_launcher");
        this.airplaneLauncherServo.setPosition(this.AIRPLANE_LOADED_POSITION);
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console

        this.movement();
        this.moveArm();
        this.grabber();
        this.airplaneLauncher();
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

        this.leftWheelMotor.setPower(leftWheelPower);
        this.rightWheelMotor.setPower(rightWheelPower);
    }

    /**
     * Controls arm movement of the robot, including both rotation and extension
     */
    public void moveArm() {
        this.rotateArm();
        this.extendArm();
    }

    public void extendArm() {
        int position = armExtensionMotor.getCurrentPosition();
        // armExtensionMax.getState() returns true when it is not being pressed
        // this will only run if the limit switch for the max arm extension has not been touched

        // if dpad_up is pressed and the max switch has not been hit
        // extend the arm
        if (gamepad2.dpad_up && armRetractionSwitch.getState()) {
            this.armExtensionMotor.setTargetPosition(position + ARM_EXTEND_SPEED);
            this.armExtensionMotor.setPower(0.5);

        } else if (gamepad2.dpad_down && armExtensionSwitch.getState()) {
            // if dpad_up is pressed and the max switch has not been hit
            // retract the arm
            this.armExtensionMotor.setTargetPosition(position - ARM_EXTEND_SPEED);
            this.armExtensionMotor.setPower(-0.5);
        }

        this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Rotates arm outward and inward
     */
    public void rotateArm() {
        // get the current position of the arm
        int position = armRotationMotor.getCurrentPosition();

        if (gamepad2.right_stick_y > 0 && position > ARM_ROTATE_MIN) {
            // if the right stick is pressed down and the arm has not reached its min
            // rotate the arm inward
            if (position - ARM_ROTATE_SPEED < ARM_ROTATE_MIN) {
                // prevent the arm from exceeding its min
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MIN);

            } else {
                // move the arm inward by ARM_ROTATE_SPEED
                this.armRotationMotor.setTargetPosition(position - ARM_ROTATE_SPEED);
            }

            this.armRotationMotor.setPower(-0.15);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.extendArmInResponse(false);

        } else if (gamepad2.right_stick_y < 0 && position < ARM_ROTATE_MAX) {
            // if the right stick is pressed up and the arm has reached its max
            // rotate the arm outward
            if (position + ARM_ROTATE_SPEED > ARM_ROTATE_MAX) {
                // prevent the arm from exceeding its max
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MAX);

            } else {
                // rotate the arm outward by ARM_ROTATE_SPEED
                this.armRotationMotor.setTargetPosition(position + ARM_ROTATE_SPEED);
            }

            this.armRotationMotor.setPower(0.15);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.extendArmInResponse(true);
        }
    }

    /**
     * As the arm rotates outward, it also retracts inward and vice versa
     * To counterbalance, the arm extends or retracts accordingly to the rotation
     * 
     * @param isRotatingOutward whether the arm is rotating outwards
     */
    public void extendArmInResponse(boolean isRotatingOutward) {
        int position = armExtensionMotor.getCurrentPosition();

        // if the arm is being rotated outward, 
        // extend the arm outward too
        if (isRotatingOutward) {
            this.armExtensionMotor.setTargetPosition(position + ARM_EXTEND_SPEED / 2);
            this.armExtensionMotor.setPower(0.1);
            
        } else { // if the arm is being rotated inward,
            // retract the arm inward too
            this.armExtensionMotor.setTargetPosition(position - ARM_EXTEND_SPEED / 2);
            this.armExtensionMotor.setPower(-0.1);
        }

        this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        this.presetGrabberRotationPositions();

        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            // if the left bumper is pressed, open the claw
            this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) {
            // if the right bumper is pressed, close the claw
            this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);
        }

        // get the current position of the claw rotation servo
        double currClawPosition = this.clawRotationServo.getPosition();

        // if the left trigger is pressed
        if (gamepad2.left_trigger > 0) {
            // rotate the claw upward
            this.clawRotationServo.setPosition(currClawPosition - this.CLAW_ROTATE_SPEED);

        } else if (gamepad2.right_trigger > 0) {
            // if the right trigger is pressed
            // rotate the claw downward
            this.clawRotationServo.setPosition(currClawPosition + this.CLAW_ROTATE_SPEED);
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
            this.clawRotationServo.setPosition(0.0);

        } else if(gamepad2.a) {
            // if A button is pressed,
            // rotate the claw downward
            this.clawRotationServo.setPosition(1.0);
        }
    }

    /**
     * Launches airplane at a fixed angle
     */
    public void airplaneLauncher() {
        // if Y button is pressed
        // move the hook backward to release the rubber band
        if (gamepad1.y) {
            this.airplaneLauncherServo.setPosition(AIRPLANE_FIRING_POSITION);
        }
    }
}
