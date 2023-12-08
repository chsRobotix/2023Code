package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Autonomous")
public class autonomous extends LinearOpMode {
    // variables for the vision system
    // Variable to store and instance of the TensorFlow Object Detection(TFOD)
    // processor.
    private TfodProcessor tfod;

    // variable to store an instane of VisionPortal
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();

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

        // drive forward and backwards to test
        setWheelPower(1.0);
        sleep(1000);
        setWheelPower(-1.0);

        telemetry.addData("", "Done running!");
    }

    /*
     * Inititialize the variables for Tfod
     */
    public void initTfod() {
        // create a TensorFlow processor
        tfod = new TfodProcessor.Builder().build();

        // create a VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();
    }

    /*
     * Set the power of the wheels to the same value
     */
    public void setWheelPower(double wheelPower) {
        this.leftWheelMotor.setPower(wheelPower);
        this.rightWheelMotor.setPower(wheelPower);
    }

    /*
     * Turns the robot a certain number of degrees
     * Negative is left
     * Positive is right
     */
    public void turn(double degrees) {
        double wheelPower = 1.0;
    }
}