package frc.robot.subsystems;

public class ArmConfig {
    /*
     * General CANSparkMax settings
     */
    public static final boolean TOP_SET_INVERTED = true;
    public static final boolean BOT_SET_INVERTED = true;

    public static final int BOT_CURRENT_LIMIT = 40;
    public static final int TOP_CURRENT_LIMIT = 40;

    /*
     * Soft limits
     */
    public static final float TOP_FORW_LIMIT = (float)Math.toRadians(190); //converts degree to radian
    public static final float TOP_REV_LIMIT = (float)Math.toRadians(7); 
    public static final boolean TOP_SOFT_LIMIT_ENABLE = true;

    public static final float BOT_FORW_LIMIT = (float)Math.toRadians(135);
    public static final float BOT_REV_LIMIT = (float)Math.toRadians(40);
    public static final boolean BOT_SOFT_LIMIT_ENABLE = true;
    
    /**
     * Encoder details
     */
    public static final boolean BOT_ENC_INVERT = true;
    public static final double BOT_ENC_GEAR_RATIO =  1.0; // Encoder is 1:1 with output shaft
    public static final double BOT_ENC_OFFSET = 50.2;
    public static final double BOT_POS_CONV_FACTOR = 2 * Math.PI / BOT_ENC_GEAR_RATIO;
    public static final double BOT_VEL_CONV_FACTOR = BOT_POS_CONV_FACTOR / 60.0;

    public static final boolean TOP_ENC_INVERT = false;
    public static final double TOP_ENC_GEAR_RATIO =  1.0; // Encoder is 1:1 with output shaft
    public static final double TOP_ENC_OFFSET = 285;
    public static final double TOP_POS_CONV_FACTOR = 2 * Math.PI / TOP_ENC_GEAR_RATIO;
    public static final double TOP_VEL_CONV_FACTOR = TOP_POS_CONV_FACTOR / 60.0;

    /**
     * ProfiledPidController settings
     */
    public static final double BOT_MAX_VEL = Math.PI * 1;
    public static final double BOT_MAX_ACCEL = Math.PI * 1;

    public static final double TOP_MAX_VEL = Math.PI * 1;
    public static final double TOP_MAX_ACCEL = Math.PI * 1;

    public static final double BOT_KP = 0;
    public static final double BOT_KI = 0;
    public static final double BOT_KD = 0;

    public static final double TOP_KP = 0;
    public static final double TOP_KI = 0;
    public static final double TOP_KD = 0;

    /**
     * Networktables 
     */
    public static final String BOT_DATA_TABLE = "Arm/BotData";
    public static final String TOP_DATA_TABLE = "Arm/TopData";
}
