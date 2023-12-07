public abstract class Constants {
    /* arm rotation */
    private static final int ARM_ROTATE_MAX = 2000;
    private static final int ARM_ROTATE_MIN = 0;
    private static final int ARM_ROTATE_SPEED = 50;

    /* arm extension */
    // constant for the speed that the arm extends and retracts with
    private static final int ARM_EXTEND_SPEED = 50;

    /* claw */
    // constants for the open and closed positions of the claw
    private static final double CLAW_OPEN_POSITION = 1.0;
    private static final double CLAW_CLOSE_POSITION = 0.075;

    // constants for how fast the claw rotates
    private static final double CLAW_ROTATE_SPEED = 0.003;
}