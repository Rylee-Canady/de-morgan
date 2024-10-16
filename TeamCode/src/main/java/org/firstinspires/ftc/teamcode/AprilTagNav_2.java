package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous

public class AprilTagNav_2 extends LinearOpMode {
  //control systems
  private Blinker control_Hub;
  private Blinker expansion_Hub_2;
  private IMU imu;
  //drive motors 
  private DcMotor BL;
  private DcMotor BR;
  private DcMotor FL;
  private DcMotor FR;
  //arm systems
  private DcMotor LArm;
  private DcMotor RArm;
  private DcMotor Elbow;
  private Servo Wrist;
  private Servo LClaw;//the claws are mismatched and I literally cannot for the life of me figure out how to fix it
  private Servo RClaw;//so this is actually the left claw
  private Servo Plane;
  //sensors
  private HardwareDevice Webcam_1;
  private DistanceSensor distSensor;
  private ColorSensor fSensor;
  private double Green;
  private double Blue;
  private double Red;
  NormalizedColorSensor colorSensor;

  //initializing the use of a webcam and also inporting the labels for tensor flow,
  //one for red object and one for blue
  private static final boolean USE_WEBCAM = true; 
  private final String[] LABELS = {"Blue", "Red"};
  
  //processors for apriltag and tensorflow
  private AprilTagProcessor aprilTag;
  private TfodProcessor tfod;
  private VisionPortal myVisionPortal;
  
  //instance variable to house which location the object is, 1 being left, 2 middle, 3 right
  //it is set as -1 to show that it has not found anything 
  private int found = -1;
  
  //indicates that this is the tag that the robot is looking for, this will change with what functions you need it for
  private int tempTagFound = 1;
    
  @Override
  public void runOpMode() {
    Wrist = hardwareMap.get(Servo.class, "Wrist");
    FR = hardwareMap.get(DcMotor.class, "FR");
    BR = hardwareMap.get(DcMotor.class, "BR");
    FL = hardwareMap.get(DcMotor.class, "FL");
    BL = hardwareMap.get(DcMotor.class, "BL");
    Elbow = hardwareMap.get(DcMotor.class, "Elbow");
    RArm = hardwareMap.get(DcMotor.class, "RArm");
    LArm = hardwareMap.get(DcMotor.class, "LArm");
    RClaw = hardwareMap.get(Servo.class, "RClaw");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    Plane = hardwareMap.get(Servo.class, "Plane");
    fSensor = hardwareMap.get(ColorSensor.class, "fSensor");
    imu = hardwareMap.get(IMU.class, "imu");
    distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
    initialize();
    initDoubleVision();
    waitForStart();
     
    if(opModeIsActive() == true){
      //set the apriltag target right after finding the location objective
      while(opModeIsActive()){
      
      findAprilTag();
      }
    }//end if opmode is true
  }//end runopmode

  private void findAprilTag(){
    //the min and max can move to accomodated for camera placement
    double minX = -.5;
    double maxX = .5;
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    for (AprilTagDetection detection : currentDetections) {  
      if(detection.id == found){
        IMUTurn(getAngle()+detection.ftcPose.yaw,.1);
        while(!(detection.ftcPose.x > minX && detection.ftcPose.x < maxX) && opModeIsActive()){
          if(detection.ftcPose.x < minX){     
            //strafe left
            strafe(-.25);        
            while(!(detection.ftcPose.x > minX && detection.ftcPose.x < maxX) && opModeIsActive()){
              telemetry.update();
              if(detection.ftcPose.x > maxX){
                break;
              }//end if
            }//end while
            strafe(0);
          }//end if
          else if(detection.ftcPose.x > maxX){
            strafe(.25);
            while(!(detection.ftcPose.x > minX && detection.ftcPose.x < maxX) && opModeIsActive()){
              telemetry.update();
              if(detection.ftcPose.x < minX){
                break;
              }//end if
            }//end while
            strafe(0);
          }//end elseif
        }//end while
        strafe(0);
        resetEncoders(); 
      }//end if found
    }//end for
  }//end findAprilTag()

  private void rampPower(double currentPower,double targetPower){
    //makes an integer for amount iterations
    int iterations = 5;
    //if the target power is greater than current power then ramp power up
    if(targetPower > currentPower){
      //creates the int for the increment or the amount that will be added to the current power
      double increment = (targetPower-currentPower)/iterations;
      //while loop to get the current power to the target power
      while(currentPower <= targetPower){
        //adds the calculated increment to the current power
        currentPower += increment;
        //int in miliseconds, controls how fast it ramps up
        sleep(200);
        //when converting to blocks this would be "Strafe with: power"
        strafe(currentPower);
      }//end while loop
    }//end if statement
    else if(targetPower < currentPower){
      //creates the int for the increment or the amount that will be subtracted from the current power
      double increment = (currentPower-targetPower)/iterations;
      //while loop to get the current power to the target power
      while(currentPower >= targetPower){
        //adds the calculated increment to the current power
        currentPower -= increment;
        //int in miliseconds, controls how fast it ramps up
        sleep(200);
        //when converting to blocks this would be "Strafe with: power"
        strafe(currentPower);
      }//end while loop
    }//end else if statement
  }//end rampPower
  
  private void findAprilTag() {
    double minX = -0.5;
    double maxX = 0.5;
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    for (AprilTagDetection detection : currentDetections) {
      if (detection.id == found) {
          double currentX = detection.ftcPose.x; // Update X coordinate
          double currentYaw = detection.ftcPose.yaw; // Update yaw angle

          IMUTurn(getAngle() + currentYaw, 0.1);
          //while current x is not greater than min x or current x is not less than max x
          while (!(currentX > minX && currentX < maxX) && opModeIsActive()) {
            aprilTag.getFreshDetections();
            urrentX = detection.ftcPose.x;
            if (currentX < minX) {
              // Turn left
              strafe(0.1);
            } else if (currentX > maxX) {
              // Turn right
              strafe(-0.1);
            }

          }
          // Stop strafing once the X coordinate is within range
          strafe(0);
          resetEncoders();
        }
      }
    telemetry.update(); // Update telemetry if needed
  }
  
    /**
     * Initialize AprilTag and TFOD.
     */
  private void initDoubleVision() {
    aprilTag = new AprilTagProcessor.Builder().build();
    aprilTag.setDecimation(3);
    TfodProcessor.Builder tensorBuilder;
    VisionPortal.Builder tensorVisionPortal;

    // First, create a TfodProcessor.Builder.
    tensorBuilder = new TfodProcessor.Builder();
    // Set the name of the file where the model can be found.
    tensorBuilder.setModelFileName("2-5-24.tflite");
    // Set the full ordered list of labels the model is trained to recognize.
    tensorBuilder.setModelLabels(LABELS);
    // Set the aspect ratio for the images used when the model was created.
    //tensorBuilder.setModelAspectRatio(16 / 9);
    // Create a TfodProcessor by calling build.
    tfod = tensorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessors(tfod, aprilTag)
        .build();
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    private void scanTfod(){
      List<Recognition> currentRecognitions = tfod.getRecognitions();

      for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double conf = (recognition.getConfidence() * 100);
      
      if(0<=x&&x<=300){
        found = 2;
      }
      else if(300<=x&&x<=640){
        found = 3;
      }
      
      }//end for
    }
    
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

/*
    private void tfodFind();{
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double conf = (recognition.getConfidence() * 100);
            
        }
    }
    */
    //puts the arm down
  public void armDown(){
    //Wrist.setPosition(1);
    sleep(50);
    Elbow.setTargetPosition(0);
    Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Elbow.setPower(0.6);
    LArm.setTargetPosition(-5);
    RArm.setTargetPosition(-5);
    LArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LArm.setPower(0.25);
    RArm.setPower(0.25);
  }//end armDown
       
  public void armSet(){
    Elbow.setTargetPosition(-100);
    Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Elbow.setPower(0.6);
  }
  //moves the arms to scoring position 
  public void score(){
    Elbow.setTargetPosition(-550);
    Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Elbow.setPower(0.5);
    LArm.setTargetPosition(-125);
    RArm.setTargetPosition(-125);
    LArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    RArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LArm.setPower(0.3);
    RArm.setPower(0.3);
    Wrist.setPosition(0.35);
  }//end score
        
  //method for keeping the code cleaner and for grouping the initialization commands        
  public void initialize() {
    FR.setDirection(DcMotor.Direction.FORWARD);
    BR.setDirection(DcMotor.Direction.FORWARD);
    FL.setDirection(DcMotor.Direction.FORWARD);
    BL.setDirection(DcMotor.Direction.FORWARD);
    FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LArm.setDirection(DcMotor.Direction.REVERSE);
    RArm.setDirection(DcMotor.Direction.FORWARD);
    LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Elbow.setDirection(DcMotor.Direction.FORWARD);
    LClaw.setDirection(Servo.Direction.REVERSE);
    LClaw.scaleRange(0.48, 0.55);
    RClaw.scaleRange(0.01, 0.09);
    Wrist.scaleRange(0.19, 0.75);
    resetEncoders();
    imu.resetYaw();
  }//end initialize
        
  //method for setting the colors to a variable
  public void scanColors(){
    Red = fSensor.red();
    Blue = fSensor.blue();
    Green = fSensor.green();
  }//end scan
       
  //method to stop the motors 
  public void noMove(){
    FL.setPower(0);
    FR.setPower(0);
    BL.setPower(0);
    BR.setPower(0);
  }//end noMove
           
  /*
  -------------------------------------------------
               Helper Methods (Start)
  -------------------------------------------------
  These methods are used many times, they help reduce redundancy
  */
    
  //Sets the mode of the motors to RUN_TO_POSITION
  //Typically used with moving for a specific duration
  public void runToPos(){
    FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }//end runToPos
    
  //Stops and resets the encoder value stored in the motors
  //Typically used to refresh the encoder values to make it easier to code
  public void resetEncoders(){
    FL.setPower(0);
    FR.setPower(0);
    BL.setPower(0);
    BR.setPower(0);
    FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }//end resetEncoders
  
  //Makes the motors turn, so the robot turns
  //Positive is left, negative is right
  public void turning(double speed){
    BL.setPower(speed);
    BR.setPower(speed);
    FL.setPower(speed);
    FR.setPower(speed);
  }//end turning
    
  //Makes the motors turn, so the robot strafes
  //Positive is left, negative is right
  public void strafe(double speed){
    FL.setPower(speed);
    FR.setPower(speed);
    BL.setPower(-speed);
    BR.setPower(-speed);
  }//end strafe
    
  //Makes the motors turn, so the robot moves
  //Positive is forward, negative is left
  public void move(double speed){
    FL.setPower(speed);
    FR.setPower(-speed);
    BL.setPower(speed);
    BR.setPower(-speed);
  }//end move
    
  //Sets the target position of the motors in preparation to move
  public void moveDuration(int duration){
    FL.setTargetPosition(-duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(-duration);
    BR.setTargetPosition(duration);
  }//end moveDuration
    
  //Sets the target position of the motors in preparation to strafe
  public void strafeDuration(int duration){
    FL.setTargetPosition(duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(-duration);
    BR.setTargetPosition(-duration);
  }//end strafeDuration
    
  //Sets the target position of the motors in preparation to turn
  public void turnDuration(int duration){
    FL.setTargetPosition(duration);
    FR.setTargetPosition(duration);
    BL.setTargetPosition(duration);
    BR.setTargetPosition(duration);
  }
    
  //Uses the turning() to set a power to the motors, so the robot moves
  //It's set up this way to reduce confusion of what is happening
  public void motorSpeed(double speed){
    turning(speed);
  }//end motorSpeed
    
  //Makes the robot wait until it is done moving
  //Then, it resets the encoder values in preparation for the next movement
  private void delayReset(){
    //Wait while the BL motor is still turning and the OP mode is still active
    while(BL.isBusy() && opModeIsActive()){
      telemetry.update();
    }//end while
    resetEncoders();
  }//end delayReset
    
  //Returns the Yaw angle of the IMU
  //Yaw is the rotation about the Y axis
  //Used to reduce redundant code
  private Double getAngle(){
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }//end getAngle
    
  //Moves the robot forward for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void forward(int duration,double speed,Boolean delay){
    moveDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end forward
    
  //The robot strafes left for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void strafeLeft(int duration,double speed,Boolean delay){
    strafeDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end strafeLeft
    
  //The robot strafes right for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void strafeRight(int duration,double speed,Boolean delay){
    strafeDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end strafeRight
    
  //Moves the robot turns left for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void turnLeft(int duration,double speed,Boolean delay){
    turnDuration(duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end turnLeft
    
  //Moves the robot turns right for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void turnRight(int duration,double speed,boolean delay){
    turnDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end turnRight
    
  //Moves the robot backwards for a set duration and speed
  //Include whether or not the robot should wait before starting the next movement
  public void backwards(int duration,double speed,boolean delay){
    moveDuration(-duration);
    runToPos();
    motorSpeed(speed);
    if(delay){
      delayReset();
    }//end if
  }//end backwards
    
    
    
  public void sweep(double angle, double speed){
    IMUTurn(25,.2);
    IMUTurn(-25,.2);
  }//end sweep
  
  //Makes the robot turn to a specified angle
  //The angle depends on the last time the IMU values were reset
  //The robot turns until the robot's angle is 0.5 off the specified angle
  public void IMUTurn(double Angle,double turnSpeed){
    double Min = Angle - 0.5;
    double Max = Angle + 0.5;
    while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
      if(getAngle() < Min){
                
        //turns left
        turning(turnSpeed);
                
        while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
          telemetry.update();
          if(getAngle() > Max){
            break;
          }//end if
        }//end while
        turning(0);
      }//end if
      
      else if(getAngle() > Max){
        //turns right
        turning(-turnSpeed);
        while(!(getAngle() > Min && getAngle() < Max) && opModeIsActive()){
          telemetry.update();
          if(getAngle() < Min){
            break;
          }//end if
        }//end while
        turning(0);
      }//end elseif
    }//end while
    
    resetEncoders();
    }//end IMUTurn
}   // end class
