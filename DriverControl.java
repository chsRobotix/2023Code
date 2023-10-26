package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Driver Control")
public class DriverControl extends OpMode {
    private DcMotor leftWheelMotor, rightWheelMotor;

    private double x; // the x position of left joystick; left and right position of the joystick
    private double y; // the y position of left joystick; up and down position of the joystick

    private double leftPower, rightPower;

    @Override
    public void init() {
        // assigning the motors variables to the configured names on the driver hub
        this.leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        this.rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // setting the direction of the motors
        this.leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // set x and y positions of the left joystick
        this.x = 0;
        this.y = 0;

        this.leftPower = 0;
        this.rightPower = 0;
        
    }


    /**
     * Controls wheel movement of the robot
     * Moves robot forward, backard, left, and right
     * according to left joystick
     */
    public void movement() {
        double turn = x;
        double drive = -y;

        // power levels
        // motor gear rotation is inversed
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftWheelMotor.setPower(leftPower);
        rightWheelMotor.setPower(rightPower);
    }

    /**
     * Controls arm movement of the robot
     *
     *
     */
    public void grabber() {

    }

    public void getControls() {
        // update x and y values of the left joystick
        this.x = gamepad1.left_stick_x;
        this.y = gamepad1.left_stick_y;
    }

    @Override
    public void loop() {
        telemetry.update(); // call-back to android console
        this.getControls();
        this.movement();

    }
}
