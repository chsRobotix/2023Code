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
    private final int ARM_ROTATE_MAX = 1000;
    private final int ARM_ROTATE_MIN = 0;
    private final int ARM_ROTATE_MID = (ARM_ROTATE_MAX + ARM_ROTATE_MIN) / 2;

    // constant for the speed that the arm rotates with
    private final double ARM_ROTATIONAL_VELOCITY = 1;

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

        // get the direction that the motor will rotate in
        // if only dpad_up is pressed, it moves forward
        // if only dpad_down is pressed, it moves backward
        int motorDirection = ((gamepad2.dpad_up) ? 1 : 0) - ((gamepad2.dpad_down) ? 1 : 0);

        if (motorDirection != 0) {
            // set the target position to the of the arm
            this.armExtensionMotor.setTargetPosition(armExtension + 100 * motorDirection);

            // move at a set speed 
            this.armExtensionMotor.setPower(this.ARM_EXTEND_SPEED * motorDirection);

            // set the arm to move to position
            this.armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Rotates the arm up and down
     */
    public void rotateArm(){
        int position = armRotationMotor.getCurrentPosition();
        if(gamepad1.left_bumper && position > ARM_ROTATE_MIN){
            if(position - 199 < ARM_ROTATE_MIN){
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MIN);

            } else {
                this.armRotationMotor.setTargetPosition(position - 100);
            }

            this.armRotationMotor.setPower(-0.5);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if(gamepad1.right_bumper && position < ARM_ROTATE_MAX){
            if(position + 199 > ARM_ROTATE_MAX){
                this.armRotationMotor.setTargetPosition(ARM_ROTATE_MAX);

            } else {
                this.armRotationMotor.setTargetPosition(position + 100);
            }

            this.armRotationMotor.setPower(0.5);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


    }

    /**
     * Rotates the arm up and down
     */
    public void rotateArm2() {
        double triggerVelocity = gamepad2.right_trigger - gamepad2.left_trigger;
        int bumperVelocity = ((gamepad2.right_bumper) ? 1 : 0) - ((gamepad2.left_bumper) ? 1 : 0);

        // how much the arm is rotated
        int armRotation = this.armRotationMotor.getCurrentPosition();

        // gradually raise or lower the arm 
        // if the right or left triggers are pressed, respectively
        if (Math.abs(triggerVelocity) > 0) {
            // get the sign of triggerVelocity
            int motorDirection = (int) (Math.signum(triggerVelocity));

            // determines target position for motor to move to
            // prevents moving beyond limits
            double targetPos = this.ARM_ROTATE_MAX / 2 + this.ARM_ROTATE_MAX / 2 * motorDirection;
            if (motorDirection < 1 && targetPos < this.ARM_ROTATE_MIN) {
                targetPos = this.ARM_ROTATE_MIN;
                motorDirection *= -1;
            }  

            else if (motorDirection > 1 && targetPos > this.ARM_ROTATE_MAX) {
                targetPos = this.ARM_ROTATE_MAX;
                motorDirection *= -1;
            }
          
            // set the power of the motor
            this.armRotationMotor.setPower(motorDirection * this.ARM_ROTATIONAL_VELOCITY);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // instantly raise or lower the arm 
        // if the right or left bumpers are pressed, respectively
        if (Math.abs(bumperVelocity) > 0) {
            // get the sign of triggerVelocity
            int direction = (int) (Math.signum(bumperVelocity));
            double targetPos = this.ARM_ROTATE_MAX / 2 + this.ARM_ROTATE_MAX / 2 * direction;

            // move the motor to a set position
            this.armRotationMotor.setTargetPosition(targetPos);
            this.armRotationMotor.setPower(bumperVelocity * this.ARM_ROTATIONAL_VELOCITY);

            // set the motor to run to position
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Moves the grabber up and down
     * Also closes and opens the claw
     */
    public void grabber() {
        // if the left bumper is pressed, open the claw
        if (gamepad2.left_bumper) {
            this.pincerServo.setPosition(this.CLAW_OPEN_POSITION);

        } else if (gamepad2.right_bumper) { // if the right bumper is pressed, close the claw
            this.pincerServo.setPosition(this.CLAW_CLOSE_POSITION);
        }

        double trigger = gamepad2.right_trigger - gamepad2.left_trigger;

        // if the right trigger is pressed
        if (gamepad2.right_trigger > 0 && gamepad2.left_trigger ) {
            // rotate the claw upward
            this.clawRotationServo.setPosition(0.0);

        } else if (gamepad2.a) { // if the A button is pressed
            // rotate the claw downward 
            this.clawRotationServo.setPosition(1.0);
        } 
    }

    public void presetArmRotationPositions(){
        if(gamepad2.a){
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MIN);
            this.armRotationMotor.setPower(-1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad2.b){
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MID);
            if(this.armRotationMotor.getCurrentPosition() < ARM_ROTATE_MID) {
                this.armRotationMotor.setPower(1.0);

            } else if(this.armRotationMotor.getCurrentPosition() > ARM_ROTATE_MID){
                this.armRotationMotor.setPower(-1.0);
            }
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad2.y){
            this.armRotationMotor.setTargetPosition(ARM_ROTATE_MAX);
            this.armRotationMotor.setPower(1.0);
            this.armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
