package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConfig {
    /*
     * General CANSparkMax settings
     */
    public static final boolean TOP_SET_INVERTED = true;
    public static final boolean BOT_SET_INVERTED = true;

    public static final int BOT_CURRENT_LIMIT = 80;
    public static final int TOP_CURRENT_LIMIT = 80;

    public static final double BOT_MOTOR_GEARING = 62.5;
    public static final double TOP_MOTOR_GEARING = 60;

    public static final double BOT_VOLTAGE_COMP = 7;
    public static final double TOP_VOLTAGE_COMP = 7;

    /*
     * Soft limits
     */
    public static final float TOP_FORW_LIMIT = (float) Math.toRadians(190); // converts degree to radian
    public static final float TOP_REV_LIMIT = (float) Math.toRadians(7);
    public static final boolean TOP_SOFT_LIMIT_ENABLE = true;

    public static final float BOT_FORW_LIMIT = (float) Math.toRadians(135);
    public static final float BOT_REV_LIMIT = (float) Math.toRadians(45);
    public static final boolean BOT_SOFT_LIMIT_ENABLE = false;

    /**
     * Encoder details
     */
    public static final boolean BOT_ENC_INVERT = true;
    public static final double BOT_ENC_GEAR_RATIO = 1.0; // Encoder is 1:1 with output shaft
    public static final double BOT_ENC_OFFSET = Math.toRadians(50.2 -10);
    public static final double BOT_POS_CONV_FACTOR = 2 * Math.PI / BOT_ENC_GEAR_RATIO;
    public static final double BOT_VEL_CONV_FACTOR = BOT_POS_CONV_FACTOR; // / 60.0;

    public static final boolean TOP_ENC_INVERT = false;
    public static final double TOP_ENC_GEAR_RATIO = 1.0; // Encoder is 1:1 with output shaft
    public static final double TOP_ENC_OFFSET = Math.toRadians(285);
    public static final double TOP_POS_CONV_FACTOR = 2 * Math.PI / TOP_ENC_GEAR_RATIO;
    public static final double TOP_VEL_CONV_FACTOR = TOP_POS_CONV_FACTOR;// / 60.0;

    /**
     * ProfiledPidController settings
     */
    public static final double BOT_MAX_VEL = Math.toRadians(90); 
    public static final double BOT_MAX_ACCEL = Math.toRadians(50);

    public static final double TOP_MAX_VEL = Math.toRadians(400);
    public static final double TOP_MAX_ACCEL = Math.toRadians(700);

    public static final double BOT_KP = 1.4;
    public static final double BOT_KI = 1.4;
    public static final double BOT_KD = 0;
    public static final double BOT_IZONE = Math.toRadians(6);

    public static final double TOP_KP = 2.4;
    public static final double TOP_KI = 2.6;
    public static final double TOP_KD = 0;
    public static final double TOP_IZONE = Math.toRadians(6);

    /**
     * Angular velocity feedforward
     */
    // Real Robot values:
    public static final SimpleMotorFeedforward BOT_SIMPLE_FF = new SimpleMotorFeedforward(0.001, 0, 0);
    public static final SimpleMotorFeedforward TOP_SIMPLE_FF = new SimpleMotorFeedforward(0.001, 1.05, 0.15) ;

    // Simulation FF values:
//     public static final SimpleMotorFeedforward BOT_SIMPLE_FF = new SimpleMotorFeedforward(0, 1.25, 0.06);
//     public static final SimpleMotorFeedforward TOP_SIMPLE_FF = new SimpleMotorFeedforward(0, 1.23, 0.02);


    /**
     * Networktables
     */
    public static final String BOT_DATA_TABLE = "Arm/BotData";
    public static final String TOP_DATA_TABLE = "Arm/TopData";

    /**
     * ArmDisplay
     */
    public static final double BOT_LENGTH = Units.inchesToMeters(27.75);
    public static final double TOP_LENGTH = Units.inchesToMeters(38.6);

    /**
     * Arm Feedforward
     */
    public static class ArmFeedforward {
        /**
         * Gravity Compensation
         */
        public static final double TOP_HORIZONTAL_VOLTAGE_NOCONE = 2.3;
        public static final double TOP_HORIZONTAL_VOLTAGE_CONE = 2.3 * 2.3/1.5;
        public static final double BOT_MOMENT_TO_VOLTAGE = 0.00001;

        public static final double LENGTH_BOT_TO_COG = 14.56;
        public static final double LENGTH_TOP_TO_COG = 28.22;

        public static final double GRAVITATIONAL_CONSTANT = 389.0886; // inches/s/s which is equal to 9.81 m/s/s
        public static final double BOT_FORCE = 11.29 * GRAVITATIONAL_CONSTANT; // 11.29 lb
        public static final double TOP_FORCE = 7.77 * GRAVITATIONAL_CONSTANT; // 7.77 lb
        public static final double CONE_FORCE = 1.21 * GRAVITATIONAL_CONSTANT; // 1.21 lb
    }

    /**
     * Setup a Arm simulation which has 2 individual Arms
     */
    public static class ArmSimulation {
        public static final boolean SIMULATE_GRAVITY = false; // SingleJointedArmSim doesn't know it's a double jointed
                                                              // arm.
        public static final double BOT_MASS_KG = Units
                .lbsToKilograms(ArmFeedforward.BOT_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);
        public static final double TOP_MASS_KG = Units
                .lbsToKilograms(ArmFeedforward.TOP_FORCE / ArmFeedforward.GRAVITATIONAL_CONSTANT);

        public static final double BOT_NOISE = 2.0 * Math.PI / 4096;
        public static final double TOP_NOISE = 2.0 * Math.PI / 4096;

        public static final SingleJointedArmSim ARM_BOT_SIM = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                BOT_MOTOR_GEARING,
                SingleJointedArmSim.estimateMOI(BOT_LENGTH, BOT_MASS_KG),
                BOT_LENGTH,
                BOT_REV_LIMIT-Math.toRadians(10),
                Math.toRadians(180),
                SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed
                                  // arm.
                Math.toRadians(90), // Starting angle
                VecBuilder.fill(BOT_NOISE) // Add noise with a small std-dev
        );

        public static final SingleJointedArmSim ARM_TOP_SIM = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                TOP_MOTOR_GEARING,
                SingleJointedArmSim.estimateMOI(TOP_LENGTH, TOP_MASS_KG),
                TOP_LENGTH,
                Math.toRadians(3),
                TOP_FORW_LIMIT+Math.toRadians(10),
                SIMULATE_GRAVITY, // Don't sim gravity. SingleJointedArmSim doesn't know it's a double jointed
                                  // arm.
                Math.toRadians(30), // Starting angle
                VecBuilder.fill(TOP_NOISE) // Add noise with a small std-dev
        );
    }
}
