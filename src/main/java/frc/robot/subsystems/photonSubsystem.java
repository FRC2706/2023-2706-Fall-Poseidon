// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import java.util.List;
import java.util.Optional;
import java.lang.Math;
import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//class
public class PhotonSubsystem extends SubsystemBase {

  //constants
  private double[] APRIL_HEIGHTS = {3.0,4.0,1.7,2.4};
  private double CAMERA_HEIGHT = 1;
  private double IMAGE_HEIGHT = 100.0;
  private double IMAGE_WIDTH = 640.0;
  private double CAMERA_FOV_YAW = 70.0;
  private double CAMERA_FOV_PITCH = 100.0;
  private Rotation2d CAMERA_PITCH = Rotation2d.fromDegrees(100);


  //height of april = 1 foot 3 and 1/4 to bottom of black from floor
  private double EXAMPLE_SIZE_HEIGHT = 104.6;
  private double EXAMPLE_DISTANCE = 1.000;


  //x is forwards, y is sideways with +y being left, rotation probobly if + left too
  private Pose2d cameraOffset = new Pose2d(new Translation2d(0.23,0.3), Rotation2d.fromDegrees(0));
  //networkTableName
  private String networkTableName = "PhotonCamera";
  //data max
  private int maxData = 10;

  //declarations
  private static PhotonSubsystem instance;
  private DoubleArrayPublisher pubSetPoint;
  private DoublePublisher pubRange, pubYaw;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(maxData);
  private LinearFilter filterX = LinearFilter.movingAverage(maxData);
  private LinearFilter filterY = LinearFilter.movingAverage(maxData);
  private int data;
  private int id;

  

  public static PhotonSubsystem getInstance(){
    if (instance == null){
      instance = new PhotonSubsystem();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private PhotonSubsystem() {
    //name of camera, change if using multiple cameras
    camera1 = new PhotonCamera("OV9281");
    //networktable publishers
    pubSetPoint = NetworkTableInstance.getDefault().getTable(networkTableName).getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    pubRange = NetworkTableInstance.getDefault().getTable(networkTableName).getDoubleTopic("Range").publish(PubSubOption.periodic(0.02));
    pubYaw = NetworkTableInstance.getDefault().getTable(networkTableName).getDoubleTopic("Yaw").publish(PubSubOption.periodic(0.02));
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    data = 0;
    id = -1;
  }

  public void reset(int desiredId) {
    filterX.reset();
    filterY.reset();
    filteryaw.reset();
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    data = 0;
    id = desiredId;
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  public boolean hasData() {
    //if data is the max that the filters hold
    return(data == maxData);
  }

  private TargetCorner tagXY(List<TargetCorner> corners) {
    double averageX = 0;
    double averageY = 0;
    for (TargetCorner c : corners){
      averageX += c.x;
      averageY += c.y;
    }
    averageX/=4;
    averageY/=4;
    return new TargetCorner(averageX, averageY);
  }

  private double range(double y) {
    y = (IMAGE_HEIGHT/2-y)*CAMERA_FOV_PITCH/IMAGE_HEIGHT;
    y = Math.toRadians(y);
    y += CAMERA_PITCH.getRadians();
    return (APRIL_HEIGHTS[id]-CAMERA_HEIGHT)/Math.tan(y);
  }
  
  private double range(List<TargetCorner> corners) {
    double heightSize = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
    return(EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE/heightSize);
  }

  private rotation2D yaw(double x) {
    x = (IMAGE_WIDTH/2-x)*CAMERA_FOV_YAW/IMAGE_WIDTH;
    Rotation2d yaw = Rotation2d.fromDegrees(x); 
  }

  private PhotonTrackedTarget biggestTarget(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget biggestTarget;
    double tallest = 0;
    for (PhotonTrackedTarget t:targets) {
      List<TargetCorner> corners = t.getDetectedCorners();
      double sizeY = corners.get(1).y-corners.get(3).y;
      if (sizeY > tallest) {
        tallest = sizeY;
        biggestTarget = t;
      }
    }
    return biggestTarget;
  }

  private Pose2d convertToField(double range, Rotation2d yaw, Pose2d odometryPose) {
    Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
    Translation2d visionXY = new Translation2d(range, yaw);
    Translation2d robotRotated = visionXY.rotateBy(cameraOffset.getRotation());
    Translation2d robotToTargetRELATIVE = robotRotated.plus(cameraOffset.getTranslation());
    Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
    return new Pose2d(robotToTarget.plus(odometryPose.getTranslation()), fieldOrientedTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera1.getLatestResult();
    if (result.hasTargets()){
      //get the swerve pose at the time that the result was gotten
      Optional<Pose2d> optPose= SwerveSubsystem.getInstance().getPoseAtTimestamp(result.getTimestampSeconds());
      //for security reasons
      if (optPose.isEmpty()){
        return;
      }
      Pose2d odometryPose = optPose.get();

      PhotonTrackedTarget target = null;
      //currently chooses lowest id if sees two april tags
      if (id == -1){
        target = result.getBestTarget();
        id = target.getFiducialId();
      } else{
        List<PhotonTrackedTarget> allTargets = result.getTargets();
        for (PhotonTrackedTarget t:allTargets){
          if (t.getFiducialId() == id){
           target = t;
           break;
          }
        }
        if (target == null){
          return;
        }
      }
      
      //get tag info
      List<TargetCorner> corners = target.getDetectedCorners();
      TargetCorner tag = tagXY(corners);
      //calculate range
      double range = range(tag.y);
      //calculate yaw
      Rotation2d yaw = yaw(tag.x);
      //convert to field quordinates
      Pose2d fieldToTarget = convertToField(range, yaw, odometryPose);
      //update rolling averages
      targetPos = new Translation2d(filterX.calculate(fieldToTarget.getX()),filterY.calculate(fieldToTarget.getY()));
      targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldToTarget.getRotation().getDegrees()));
      data ++;

      //publish to networktables
      pubSetPoint.accept(new double[]{targetPos.getX(),targetPos.getY(),targetRotation.getRadians()});
      pubRange.accept(range);
      pubYaw.accept(yaw.getDegrees);
    }
  }
}
