package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class DriverControl extends OpMode {
    private DcMotor leftWheelMotor, rightWheelMotor, armMotor;

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
        this.armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // setting the direction of the motors
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightWheelMotor and armMotor are forward by default

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

    public void grabber() {
        /*
         * Controls arm movement of the robot
         */


        // set armPower
        armPower = Range.clip(this.rightStickY, -1.0, 1.0);

        armMotor.setPower(armPower);
    }

    public void getControls() {
        // update x and y values of the left joystick
        this.leftStickX = gamepad1.left_stick_x;
        this.leftStickY = gamepad1.left_stick_y;

    }