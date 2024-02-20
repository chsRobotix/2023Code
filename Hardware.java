package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.vision.*;

public class Hardware {
    private OpMode opMode;

    /* wheel movement */
    // constant for the sensitivity of turning
    public final double TURNING_SENSITIVITY = 0.5;

    // the DC motors for the wheels
    public DcMotor leftWheelMotor, rightWheelMotor;

    /* arm rotation */
    // constants for how far the arm can rotate outward and inward
    public final int ARM_ROTATE_MAX = 2000;
    public final int ARM_ROTATE_MIN = 0;
    public final int ARM_ROTATE_SPEED = 50;

    // the DC motors for the arm
    public DcMotor armRotationMotor;

    /* arm extension */
    // constant for the speed that the arm extends and retracts with
    public final int ARM_EXTEND_SPEED = 50;

    // DC motor for extending the arm
    public DcMotor armExtensionMotor;

    // the limit switches to prevent the arm from extending or retracting too far
    public DigitalChannel armExtensionSwitch, armRetractionSwitch;

    /* claw pincers */
    // constants for the open and closed positions of the claw
    public final double CLAW_OPEN_POSITION = 1.0;
    public final double CLAW_CLOSE_POSITION = 0.075;

    // the servo motors for the pincers of the claw
    public Servo pincerServo;

    /* claw rotation */
    // constant for how fast the claw rotates
    public final double CLAW_ROTATE_SPEED = 0.003;

    // the servo that rotates the claw back and forth
    public Servo clawRotationServo;

    // preset positions for claw rotations
    public final double CLAW_ROTATION_LOWEST_POSITION = 0.6;
    public final double CLAW_ROTATION_HIGHEST_POSITION = 0.0;

    /* airplane */
    // starting and ending position for airplane launcher
    public final double AIRPLANE_LOADED_POSITION = 1.0 ;
    public final double AIRPLANE_FIRING_POSITION = 0.0;

    // the servo that launches the airplane
    public Servo airplaneLauncherServo;

    // limit switch on the claw preventing it from going down too much
    public DigitalChannel pincerLimiter;

    // sensors
    //public AnalogInput potentiometer;

    /* vision system */
    // Variable to store and instance of the TensorFlow Object Detection(TFOD)
    // processor.
    // public TfodProcessor tfod;

    // variable to store an instane of VisionPortal
    public VisionPortal visionPortal;

    public Hardware(OpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initializes all the robot's hardware(motors, servos, sensors, etc.)
     */
    public void init() {
        /* wheel movement */
        // assigning the motors variables to the configured names on the driver hub
        leftWheelMotor = opMode.hardwareMap.get(DcMotor.class, "left_motor");
        rightWheelMotor = opMode.hardwareMap.get(DcMotor.class, "right_motor");

        // setting the direction of the motors
        // rightWheelMotor is forward by default
        leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* arm rotation */
        armRotationMotor = opMode.hardwareMap.get(DcMotor.class, "arm_rotator");
        armRotationMotor.resetDeviceConfigurationForOpMode();

        // limiter switch at the end of the claw
        // prevents arm from smashing against the ground
        pincerLimiter = opMode.hardwareMap.get(DigitalChannel.class, "pincerLimiter");

        /* arm extension */
        armExtensionMotor = opMode.hardwareMap.get(DcMotor.class, "arm_extender");
        armExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // allows extension motor to coast while not in use
        // prevents arm from retracting during rotation
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armRetractionSwitch = opMode.hardwareMap.get(DigitalChannel.class, "armExtensionMax");
        armExtensionSwitch = opMode.hardwareMap.get(DigitalChannel.class, "armExtensionMin");

        /* claw */
        pincerServo = opMode.hardwareMap.get(Servo.class, "pincer_servo");
        pincerServo.setPosition(CLAW_CLOSE_POSITION);

        // set the servo position of the grabber rotator to prevent ground collision
        clawRotationServo = opMode.hardwareMap.get(Servo.class, "pincer_rotation_servo");
        clawRotationServo.setPosition(CLAW_ROTATION_LOWEST_POSITION);

        /* airplane launcher */
        airplaneLauncherServo = opMode.hardwareMap.get(Servo.class, "airplane_launcher");
        airplaneLauncherServo.setPosition(AIRPLANE_LOADED_POSITION);

        /* sensors */
        //potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");

        opMode.telemetry.addData("Hardware.java: ", "Initialized hardware");
        opMode.telemetry.update();
    }

    /**
     * Set the power of the wheels to the same value
     *
     * @param wheelPower Specifies the wheel power.
     *                   Positive drives the robot forward. Negative drives it backward.
     */
    public void drive(double wheelPower) {
        leftWheelMotor.setPower(-wheelPower);
        rightWheelMotor.setPower(-wheelPower);
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