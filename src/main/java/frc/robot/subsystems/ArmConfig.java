package frc.robot.subsystems;

public class ArmConfig {
    public static final boolean TOP_SET_INVERTED = true;
    public static final boolean BOTTOM_SET_INVERTED = true;

    public static final int CURRENT_LIMIT = 60;

    //soft limit constants for top arm
    public static final float top_arm_forward_limit = (float)Math.toRadians(190); //converts degree to radian
    public static final float top_arm_reverse_limit = (float)Math.toRadians(7); 
    public static final boolean TOP_SOFT_LIMIT_ENABLE = true;

    //soft limit constant for bottom arm
    public static final float bottom_arm_forward_limit = (float)Math.toRadians(135);
    public static final float bottom_arm_reverse_limit = (float)Math.toRadians(40);
    public static final boolean BOTTOM_SOFT_LIMIT_ENABLE = true;

    //duty cycle encoders
    public static final int bottom_duty_cycle_channel = 7;

}
