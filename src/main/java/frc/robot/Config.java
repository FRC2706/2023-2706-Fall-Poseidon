package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.RobotBase;

public final class Config {
  /**
   * Instructions for set up of robot.conf file on robot
   *
   * 0. Connect to the robot to the robot using a usb cable or the wifi network.
   * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local
   * (ssh admin@roboRIO-2706-FRC.local)
   * a. There is no password on a freshly flashed roboRIO
   * 2. Go up a directory (cd ..)
   * 3. cd into lvuser/ (cd lvuser/)
   * 4. Create a new file called robot.conf (touch robot.conf)
   * 5. Open the file with vi (vi robot.conf)
   * 6. Press i to enter insert mode
   * 7. Add an integer denoting the robot id. If it's the first robot, use 0,
   * second use 1 etc.
   * 8. Press [ESC] followed by typing :wq in order to save and quit
   * 9. To verify this worked type: more robot.conf
   * 10. If it displays the value you entered, it was successful
   * 11. Type exit to safely exit the ssh session
   */

  private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");

  /**
   * ID of the robot that code is running on
   */
  private static int robotId = -1;

  /**
   * CAN IDs, ports, channels, etc.
   */
  public static class CANID {
    // Arm Subsystem
    public static final int TOP_ARM_SPARK_CAN_ID = robotSpecific(5,0,0,0,0,18,18);
    public static final int BOTTOM_ARM_SPARK_CAN_ID = robotSpecific(4,0,0,0,0,19,19);

    //Arm Subsystem
    public static final int BOTTOM_CANCODER_CAN_ID = 2;
    

  }

  /**
   * Returns one of the values passed based on the robot ID
   *
   * @param first The first value (default value)
   * @param more  Other values that could be selected
   * @param <T>   The type of the value
   * @return The value selected based on the ID of the robot
   */
  @SafeVarargs
  public static <T> T robotSpecific(T first, T... more) {
    if (getRobotId() < 1 || getRobotId() > more.length) {
      return first;
    } else {
      return more[getRobotId() - 1];
    }
  }

  /**
   * Obtain the robot id found in the robot.conf file
   *
   * @return The id of the robot
   */
  public static int getRobotId() {

    if (robotId < 0) {
      // Set to the ID of the 2023 Competition robot if the simulation is running
      if (RobotBase.isSimulation()) {
        robotId = 0;

        // Not simulation so read the file on the roborio for it's robot id.
      } else {
        try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
          robotId = Integer.parseInt(reader.readLine());
        } catch (Exception e) {
          robotId = 0; // DEFAULT TO COMP ROBOT IF NO ID IS FOUND
        }
      }
    }

    return robotId;
  }

  /**
   * ROBOT IDs
   * 
   * ID 0: Poseidon (Charged Up)
   * ID 1: Clutch (Rapid React)
   * ID 2: Beetle (Small Talon tank drive)
   * ID 3: Cosmobot (Deep Space)
   * 
   * ...
   * 
   * 
   **/




   /** ADD CONSTANTS BELOW THIS LINE */

   //PCM Can ID 
   public static final int CTRE_PCM_CAN_ID = 1;

   //Constants for arm pneumatics 
   public static final int ARMLOW_PNEUMATIC_FORWARD_CHANNEL=0;
   public static final int ARMLOW_PNEUMATIC_REVERSE_CHANNEL = 1;
   
  public static final int ARMHIGH_PNEUMATIC_FORWARD_CHANNEL = 2;
  public static final int ARMHIGH_PNEUMATIC_REVERSE_CHANNEL = 3;
  
  //For intake pneumatics
  public static final int INTAKE2_PNEUMATIC_FORWARD_CHANNEL = 5;
  public static final int INTAKE2_PNEUMATIC_REVERSE_CHANNEL = 7;
  public static final int INTAKE1_PNEUMATIC_FORWARD_CHANNEL = 4;
  public static final int INTAKE1_PNEUMATIC_REVERSE_CHANNEL = 6;
}
