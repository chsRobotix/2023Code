package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/*
 * Allows drivercontrol and autonomous to share constants and motors
 */
public class Robot {
    /* wheel movement */
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

    // the limit switches for arm extension and retraction
    public DigitalChannel armExtensionSwitch;
    public DigitalChannel armRetractionSwitch;

    /* claw */
    // constants for the open and closed positions of the claw
    public final double CLAW_OPEN_POSITION = 1.0;
    public final double CLAW_CLOSE_POSITION = 0.075;

    // the servo motors for the pincers of the claw
    public Servo pincerServo;

    // the limit switches for the pincers of the claw
    public DigitalChannel clawOpenSwitch;
    public DigitalChannel clawCloseSwitch;

    // constants for how fast the claw rotates
    public final double CLAW_ROTATE_SPEED = 0.003;

    // the servo that rotates the claw back and forth
    public Servo clawRotationServo;    

    /* airplane */
    // starting and ending position for airplane launcher
    public final double AIRPLANE_LOADED_POSITION = 1.0;
    public final double AIRPLANE_FIRING_POSITION = 0.5;

    // the servo that launches the airplane
    public Servo airplaneLauncherServo;

    public Robot() {
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
        clawOpenSwitch = hardwareMap.get(Servo.class, "claw_max");
        clawCloseSwitch = hardwareMap.get(Servo.class, "claw_min");

        pincerServo.setPosition(this.CLAW_CLOSE_POSITION);

        // set the servo position of the grabber rotator to prevent ground collision
        clawRotationServo = hardwareMap.get(Servo.class, "pincer_rotation_servo");
        clawRotationServo.setPosition(0.0);

        /* airplane */
        // set the servo position of airplaneLauncherServo to stretch rubber band
        airplaneLauncherServo = hardwareMap.get(Servo.class, "airplane_launcher");
        airplaneLauncherServo.setPosition(this.AIRPLANE_LOADED_POSITION);
    }
}