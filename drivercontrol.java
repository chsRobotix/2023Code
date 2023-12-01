package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;

@TeleOp(name = "Driver Control")
public class drivercontrol extends OpMode {
    /* wheel movement */
    // the DC motors for the wheels
    private DcMotor leftWheelMotor, rightWheelMotor;

    /* arm rotation */
    // constants for how far the arm can rotate outward and inward
    private final int ARM_ROTATE_MAX = 2000;
    private final int ARM_ROTATE_MIN = 0;
    private final int ARM_ROTATE_MID = (this.ARM_ROTATE_MAX + this.ARM_ROTATE_MIN) / 2;
    private final int ARM_ROTATE_SPEED = 100;

    // the DC motors for the arm
    private DcMotor armRotationMotor;

    /* arm extension */
    // constants for how far the arm can extend and retract
    private final int ARM_EXTEND_LIMIT = 0;
    private final int ARM_RETRACT_LIMIT = 0;

    // constant for the speed that the arm extends and retracts with
    private final double ARM_EXTEND_SPEED = 50;
    private DcMotor armExtensionMotor;

    /* claw */
    // constants for the open and closed positions of the claw
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSE_POSITION = 0.075;

    // constants for how fast the claw rotates
    private final double CLAW_ROTATE_SPEED = 0.003;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    // the servo motors for the pincers of the claw
    private Servo pincerServo;

    /* airplane */
    // starting and ending position for airplane launcher
    private final double AIRPLANE_LOADED_POSITION = 1.0;
    private final double AIRPLANE_FIRING_POSITION = 0.5;

    // the servo that launches the airplane
    private Servo airplaneLauncherServo;

    // private TouchSensor armExtensionMin;
    private TouchSensor armExtensionMax;

    @Override
    public void init() {
        /* wheel movement */
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        this.armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotator");
        this.armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extender");

        this.pincerServo = hardwareMap.get(Servo.class, "pincer_servo");
        this.clawRotationServo = hardwareMap.get(Servo.class, "pincer_rotation_servo");

        this.airplaneLauncherServo = hardwareMap.get(Servo.class, "airplane_launcher");

        this.armExtensionMax = hardwareMap.get(TouchSensor.class, "armExtensionMax");
        // this.armExtensionMin = hardwareMap.get(TouchSensor.class, "armExtensionMin");

        // setting the direction of the motors
        // rightWheelMotor and armRotationMotor are forward by default
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

        /* claw */
        // setting the two pincer servo positions to open
        this.pincerServo = hardwareMap.get(Servo.class, "pincer_servo");
        this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        // set the servo position of the grabber rotator to prevent ground collision
        this.clawRotationServo = hardwareMap.get(Servo.class, "pincer_rotation_servo");
        this.clawRotationServo.setPosition(0);

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
        //telemetry.addData("Arm rotation position ", armRotationMotor.getCurrentPosition());
        //telemetry.addData("right stick y ", gamepad2.right_stick_y);
        telemetry.addData("Sensor is pressed: ", armExtensionMax.isPressed());
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
/*    public void extendArm() {
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
    }*/

    public void extendArm(){
        int position = armExtensionMotor.getCurrentPosition();

        if (gamepad2.dpad_up) { //  && !armExtensionMax.isPressed()
            this.armExtensionMotor.setTargetPosition(position + 50);
            this.armExtensionMotor.setPower(0.3);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.dpad_down ) { // && !armExtensionMin.isPressed()
            this.armExtensionMotor.setTargetPosition(position - 50);
            this.armExtensionMotor.setPower(-0.3);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Rotates arm outward and inward
     */
    public void rotateArm() {
        // get the current position of the arm
        int position = armRotationMotor.getCurrentPosition();

        // if the right stick is pressed down and the arm has not reached its min
        if (gamepad2.right_stick_y > 0 && position > ARM_ROTATE_MIN) {
            // rotate the arm inward

            // prevent the arm from exceeding its min
            if (position - 200 < ARM_ROTATE_MIN) {
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MIN);

            } else {
                // move the arm inward by 100
                this.armRotationMotor.setTargetPosition(position - 100);
            }

            this.armRotationMotor.setPower(-0.1);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.extendArmInResponse(false);

        } else if (gamepad2.right_stick_y < 0 && position < ARM_ROTATE_MAX) {
            // if the right stick is pressed up and the arm has reached its max
            // rotate the arm outward

            // prevent the arm from exceeding its max
            if (position + 200 > ARM_ROTATE_MAX) {
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MAX);

            } else {
                // rotate the arm outward by 100
                this.armRotationMotor.setTargetPosition(position + 100);
            }

            this.armRotationMotor.setPower(0.1);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.extendArmInResponse(true);
        }
    }

    /**
     * Moves the arm to hard-coded positions of min, mid, and max
     * using buttons on gamepad2
     */
    /*public void presetArmRotationPositions() {
        // if the A button is pressed,
        // move the arm to the minimum position
        if (gamepad2.a) {
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MIN);
            this.armRotationMotor.setPower(-1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // if the B button is pressed
        // move the arm to the midpoint
        if (gamepad2.b) {
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MID);
            if (this.armRotationMotor.getCurrentPosition() < ARM_ROTATE_MID) {
                this.armRotationMotor.setPower(1.0);

            } else if (this.armRotationMotor.getCurrentPosition() > ARM_ROTATE_MID) {
                this.armRotationMotor.setPower(-1.0);
            }

            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // if the Y button is pressed,
        // move the arm to its max
        if (gamepad2.y) {
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MAX);
            this.armRotationMotor.setPower(1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }*/

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        this.presetGrabberRotationPositions();

        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) { // if the right bumper is pressed, close the claw
            this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);
        }

        // if the left trigger is pressed
        if (gamepad2.left_trigger > 0) {
            // rotate the claw upward
            double currClawPosition = this.clawRotationServo.getPosition();
            this.clawRotationServo.setPosition(currClawPosition - this.CLAW_ROTATE_SPEED);

        } else if (gamepad2.right_trigger > 0) { // if the right trigger is pressed
            // rotate the claw downward
            double currClawPosition = this.clawRotationServo.getPosition();
            this.clawRotationServo.setPosition(currClawPosition + this.CLAW_ROTATE_SPEED);
        }
    }

    public void presetGrabberRotationPositions() {
        if (gamepad2.y) {
            this.clawRotationServo.setPosition(0.0);

        } else if(gamepad2.b){
            this.clawRotationServo.setPosition(0.5);

        } else if(gamepad2.a){
            this.clawRotationServo.setPosition(1.0);

        }
    }

    public void airplaneLauncher() {
        if (gamepad1.y) {
            this.airplaneLauncherServo.setPosition(AIRPLANE_FIRING_POSITION);
        }
    }

    // There appears to be a mechanical problem where
    // as the arm rotates outward, the arm extends inward. And as
    // the arm rotates inward, the arm extends outward. To fix this problem
    // this function is called during the arm rotation function to counteract the
    // problem
    public void extendArmInResponse(boolean isMovingOutward) {
        int position = armExtensionMotor.getCurrentPosition();

        // if the arm is being rotated outward,
        // extend the arm outward too
        if (isMovingOutward) {
            this.armExtensionMotor.setTargetPosition(position + 25);
            this.armExtensionMotor.setPower(0.1);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else { // if the arm is being rotated inward,
            // retract the arm inward too
            this.armExtensionMotor.setTargetPosition(position - 25);
            this.armExtensionMotor.setPower(-0.1);
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
