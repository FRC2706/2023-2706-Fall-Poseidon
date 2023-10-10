package frc.robot.subsystems;

import frc.robot.Config;

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

    //arm offsets
    public static final double bottom_arm_offset = 307.800000;

    //syncing encoders
    public static double ENCODER_SYNCING_TOLERANCE = 0.01; //radians

    public static final double BOTTOM_NEO_GEAR_RATIO =  Config.robotSpecific(62.5,0.0,0.0,0.0);

    public static final double bottomArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
    public static final double bottomArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;

}
