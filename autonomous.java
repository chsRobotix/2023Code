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

    // the servo motors for the pincers of the claw
    private Servo pincerServo;

    // the limit switches for the pincers of the claw
    private DigitalChannel clawOpenSwitch;
    private DigitalChannel clawCloseSwtich;

    // constants for how fast the claw rotates
    private final double CLAW_ROTATE_SPEED = 0.003;

    // the servo that rotates the claw back and forth
    private Servo clawRotationServo;

    /* airplane */
    // starting and ending position for airplane launcher
    private final double AIRPLANE_LOADED_POSITION = 1.0;
    private final double AIRPLANE_FIRING_POSITION = 0.5;

    // the servo that launches the airplane
    private Servo airplaneLauncherServo;

    /* vision system */
    // Variable to store and instance of the TensorFlow Object Detection(TFOD)
    // processor.
    private TfodProcessor tfod;

    // variable to store an instane of VisionPortal
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        /* wheel movement */
        // assigning the motors variables to the configured names on the driver hub
        leftWheelMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightWheelMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // setting the direction of the motors
        // rightWheelMotor is forward by default
        leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* arm rotation */
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotator");
        armRotationMotor.resetDeviceConfigurationForOpMode();

        /* arm extension */
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extender");
        armExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // allows extension motor to coast while not in use
        // prevents arm from retracting during rotation
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armRetractionSwitch = hardwareMap.get(DigitalChannel.class, "armExtensionMax");
        armExtensionSwitch = hardwareMap.get(DigitalChannel.class, "armExtensionMin");

        /* claw */
        pincerServo = hardwareMap.get(Servo.class, "pincer_servo");

        pincerServo.setPosition(this.CLAW_CLOSE_POSITION);

        // set the servo position of the grabber rotator to prevent ground collision
        clawRotationServo = hardwareMap.get(Servo.class, "pincer_rotation_servo");
        clawRotationServo.setPosition(0.0);

        /* airplane */
        // set the servo position of airplaneLauncherServo to stretch rubber band
        airplaneLauncherServo = hardwareMap.get(Servo.class, "airplane_launcher");
        airplaneLauncherServo.setPosition(this.AIRPLANE_LOADED_POSITION);

        // drive forward and backwards to test
        setWheelPower(1.0);
        sleep(1000);
        setWheelPower(-1.0);

        telemetry.addData("", "Done running!");
    }

    /**
     * Inititialize the variables for Tfod
     */
    public void initTfod() {
        // create a TensorFlow processor
        tfod = new TfodProcessor.Builder().build();

        // create a VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();
    }

    /**
     * Set the power of the wheels to the same value
     */
    public void setWheelPower(double wheelPower) {
        robot.leftWheelMotor.setPower(wheelPower);
        robot.rightWheelMotor.setPower(wheelPower);
    }

    /**
     * Turns the robot a certain number of degrees
     * Negative is left
     * Positive is right
     */
    public void turn(double degrees) {
        double wheelPower = 1.0;
    }
}