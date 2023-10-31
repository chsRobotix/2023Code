package org.firstinspires.ftc.teamcode;

import java.io.Serial;
import java.rmi.server.ServerCloneException;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class DriverControl extends OpMode {
    // the DC motors for the wheels
    private DcMotor leftWheelMotor, rightWheelMotor;
    
    // the DC motors for the arm 
    private armRotationMotor, armExtensionMotor;

    // the servo motors for the pincers of the claw
    private Servo leftPincerServo, rightPincerServo;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    private double leftStickX; // the x position of left joystick; left and right position of the joystick
    private double leftStickY; // the y position of left joystick; up and down position of the joystick

    private double rightStickX; // x position of right joystick
    private double rightStickY; // y position of the right joystick

    private double leftWheelPower, rightWheelPower, armPower;

    @Override
    public void init() {
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");
        this.armRotationMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        this.leftPincerServo = hardwareMap.get(Servo.class, "left_pincer_servo");
        this.rightPincerServo = hardwareMap.get(Servo.class, "right_pincer_servo");

        // setting the direction of the motors
        // rightWheelMotor and armRotationMotor are forward by default
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // setting the two servo positions to 1, which is upright, aka not pressing the claw
        leftPincerServo.setPosition(1.0);
        rightPincerServo.setPosition(1.0);

        // set x and y positions of the left joystick
        this.leftStickX = 0;
        this.leftStickY = 0;

        // set x and y positions of right joystick
        this.rightStickX = this.rightStickY = 0;

        this.leftWheelPower = 0;
        this.rightWheelPower = 0;

        this.armPower = 0;
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console
        this.getControls();

        this.movement();
        this.armMovement();
        this.grabber();
    }

    public void movement() {
        /*
         * Controls wheel movement of the robot
         * Moves robot forward, backard, left, and right
         * according to left joystick
         */
        double turn = this.leftStickX;
        double drive = this.leftStickY;

        // power levels
        // motor gear rotation is inversed
        leftWheelPower = Range.clip(drive + turn, -1.0, 1.0);
        rightWheelPower = Range.clip(drive - turn, -1.0, 1.0);

        leftWheelMotor.setPower(leftWheelPower);
        rightWheelMotor.setPower(rightWheelPower);
    }

    public void armMovement() {
        /*
         * Controls arm movement of the robot
         */
        
        // set armPower
        armPower = Range.clip(this.rightStickY, -1.0, 1.0);

        // set the motor to the power
        armRotationMotor.setPower(armPower);
    }

    public void grabber() {
        // if A button is pressed, release the claw
        if (gamepad1.a) {
            this.leftPincerServo.setPosition(1.0);
        }

        // if B button is pressed, close the claw to half
        if (gamepad1.b) {
            this.leftPincerServo.setPosition(0.5);
        }
    }

    public void getControls() {
        // update x and y values of the left joystick
        this.leftStickX = gamepad1.left_stick_x;
        this.leftStickY = gamepad1.left_stick_y;

    }
}
