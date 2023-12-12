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
    public DigitalChannel clawCloseSwtich;

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
}