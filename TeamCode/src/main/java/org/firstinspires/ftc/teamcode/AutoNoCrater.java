package org.firstinspires.ftc.teamcode;
//V3.0
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="No Crater", group="4719")

public class AutoNoCrater extends LinearOpMode
{
    //variables
    private Hardware robot = new Hardware();
    private GoldAlignDetector detector;
    private ElapsedTime     runtime = new ElapsedTime();
    private int RFPos;


    @Override
    public void runOpMode() {
        //initialize
        robot.init(hardwareMap);
        telemetry.addData("Status: ", "Starting Up");
        telemetry.update();
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.maxAreaScorer.weight = 0.005; //
        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        detector.enable(); // Start the detector!
        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        //wait for the button to be pressed
        waitForStart();
            //wait for the gyro sensor to be calibrated
           while (robot.mrGyro.isCalibrating() && opModeIsActive()) {
           }

            robot.hingeMotor.setPower(1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //if the robots switch is not pressed, keep on running the motor
            while (!robot.cdi.getDigitalChannelState(1) && opModeIsActive()) {
               //motor power while coming down
                robot.hingeMotor.setPower(.1);
            }
            //if the robots switch is pressed then stop the robot and get out of this while loop
            if (robot.cdi.getDigitalChannelState(1) && opModeIsActive()) {
               //set power to 0, stop.
                robot.hingeMotor.setPower(0);
            }
            //unstick the robot from the wall, move back from the lander
            driveBack(.8, 75);
            //move left to get out of the hinge
            driveLeft(.8, 600);
            // turn to make sure the robot is lined up
            turn(0);
            //move back and turn the camera to face the blocks
            driveStraightBack(1, 1400, -90);
            driveStraightBack(1, 1500, -90);
            //go forward until it detects the cube
            while (!detector.getAligned() && opModeIsActive()) {
                //forward
                robot.rightFront.setPower(.6);
                robot.leftBack.setPower(.6);
            }
            //when detected, knock it off
            //short stop
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            sleep(50);
            //store position
            RFPos = robot.rightFront.getCurrentPosition();
            //make sure the robot is perfect parallel to the cube
            turn(-90);
            //knock it off
            driveRight(.9, 1200);
            //make sure the robot is parallel to knocked off cube
            turn(-90);
            //come back after knock down
            driveLeft(.9, 1200);
            //make sure the robot is straight
            turn(-90);
            //disable the detector
            detector.disable();
            //go forward the distance needed, towards vuforia
            driveStraightRemDist(1, 6300, -45);
            //go left to hit the wall
            driveRight(.9, 1500);
            //come off the wall
            driveLeft(.6, 90);
            //make sure the robot is parallel to the wall
            turn(-45);
            //go to depot
            driveStraightBack(1, 3600, -45);
            //drop off the team marker
            robot.drop.setPosition(1);
            sleep(500);
            robot.drop.setPosition(0);
            sleep(500);
            turn(-45);
            //go park bro
            driveStraightForwardFinal(1, 9000);

        }


    public void turnAbsolute(int target) {
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
        robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.3;

        while (Math.abs(robot.zAccumulated - target) > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (robot.zAccumulated > target) {  //if gyro is positive, we will turn right
                robot.rightFront.setPower(-turnSpeed);
                robot.leftFront.setPower(turnSpeed);
                robot.rightBack.setPower(-turnSpeed);
                robot.leftBack.setPower(turnSpeed);      //turn left
            }

            if (robot.zAccumulated < target) {  //if gyro is positive, we will turn left
                robot.rightFront.setPower(turnSpeed);
                robot.leftFront.setPower(-turnSpeed);
                robot.rightBack.setPower(turnSpeed);     //turn right
                robot.leftBack.setPower(-turnSpeed);
            }
            telemetry.addData("Degree", String.format("%03d", robot.zAccumulated));
            telemetry.update();
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        }

        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);     //turn off
        robot.leftBack.setPower(0);
    }

    public void turn(int degrees) {
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turnAbsolute(degrees);
    }

    public void driveRight(double speed, int Target) {
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
        //RESET encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.leftFront.setTargetPosition(Target);
        robot.rightBack.setTargetPosition(Target);
        //run to position
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set speed
        robot.leftFront.setPower(speed);
        robot.rightBack.setPower(speed);
        //update telemetry to show positions
        while (robot.leftFront.isBusy() && robot.rightBack.isBusy() && opModeIsActive()) {
        }
        //turn off any extra power
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
        //turn on encoders
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveLeft(double speed, int Target) {
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
        //RESET encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.leftFront.setTargetPosition(-Target);
        robot.rightBack.setTargetPosition(-Target);
        //run to position
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        robot.leftFront.setPower(speed);
        robot.rightBack.setPower(speed);
        //update telemetry to show positions
        while (robot.leftFront.isBusy() && robot.rightBack.isBusy()&& opModeIsActive()) {
        }
        //turn off any extra power
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        //turn on encoders
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveStraightForwardFinal( double power, int duration) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setTargetPosition(duration);
        robot.leftBack.setTargetPosition(duration);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftBack.getCurrentPosition() < duration && opModeIsActive()) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftFSpeed = 0 + (robot.zAccumulated-target)/ 100;
            rightBSpeed = 0 - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightBSpeed);
            robot.leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.addData("Angle: ", robot.zAccumulated);
            telemetry.addData("3. Distance to go", duration + startPosition - robot.leftBack.getCurrentPosition());
            telemetry.update();
        }

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
    public void driveBack(double speed, int Target) {
        //RESET encoders
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.rightFront.setTargetPosition(-Target);
        robot.leftBack.setTargetPosition(-Target);
        //run to position
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        //update telemetry to show positions
        while (robot.rightFront.isBusy() && robot.leftBack.isBusy()&& opModeIsActive()) {

        }
        //turn off any extra power
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        //turn on encoders
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
        private void driveStraightBack( double power, int duration, int desiredDegree) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotor.Direction.FORWARD);

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setTargetPosition(duration);
        robot.leftBack.setTargetPosition(duration);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftBack.getCurrentPosition() < duration && opModeIsActive()) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftFSpeed = 0 + (robot.zAccumulated-target)/ 100;
            rightBSpeed = 0 - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightBSpeed);
            robot.leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.addData("Angle: ", robot.zAccumulated);
            telemetry.addData("3. Distance to go", duration + startPosition - robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
        turn(desiredDegree);

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
    public void driveStraightRemDist(double speed, int Target, int desiredDegree){

        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int dist = Target - RFPos;

        robot.rightFront.setTargetPosition(dist);
        robot.leftBack.setTargetPosition(dist);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftBack.getCurrentPosition() < dist && opModeIsActive()) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = speed + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = speed - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftFSpeed = 0 + (robot.zAccumulated-target)/ 100;
            rightBSpeed = 0 - (robot.zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightBSpeed);
            robot.leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.addData("Angle: ", robot.zAccumulated);
            telemetry.addData("3. Distance to go", dist + startPosition - robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
        //turn it off
        turn(desiredDegree);

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
}

