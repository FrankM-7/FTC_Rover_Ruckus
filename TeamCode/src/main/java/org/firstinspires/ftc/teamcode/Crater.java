package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Hardware;
import org.opencv.core.Range;

@Autonomous(name="Crater", group="DogeCV")

public class Crater extends LinearOpMode
{
    Hardware robot = new Hardware();
    // Detector object
    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    int RFPos;
    int x=1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings
        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //
        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        waitForStart();

        while (robot.mrGyro.isCalibrating()&& opModeIsActive()) {
        }
        driveStraightForward(30000, 0.4);
        //move back
        //hinge(.8, -7500);
       /* driveBack(.9, 75);
        turn(0);
        //move left
        driveLeft(.9, 500);
        turn(0);
        //move back
        driveBack(.9, 1500);
        turn(0);
        //turn right
        turn(-90);
        driveBack(.9, 1500);
        turn(-90);
        sleep(500);
        while (!detector.getAligned() && opModeIsActive()) {
            //move forward until it reads cube
            robot.rightFront.setPower(.6);
            robot.leftBack.setPower(.6);

        }
        while (detector.getAligned() && x==1 && opModeIsActive()) {//when it reads a yellow square
            //stop motors
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            sleep(250);
            //store position
            RFPos = robot.rightFront.getCurrentPosition();
            turn(-90);
            //knock it off
            driveRight(.9, 1000);
            turn(-90);
            //come back
            driveLeft(.9, 800);
            turn(-90);
            x=2;
        }
        //go forward the distance needed
        remDist(.9, 6000);
        turn(-225);

        driveLeft(.9, 800);
        turn(-225);
        driveRight(.9, 200);
        turn(-225);
        driveBack(.9, 3200);
        turn(-225);
        robot.drop.setPosition(1);
        sleep(500);
        robot.drop.setPosition(0);
        sleep(500);
        driveForward(.9, 3000);
        turn(-225);
        driveForward(1, 1500);
        driveLeft(.9, 300);
        driveForward(1,1000);
        //  turn(-270);
        //diagonalLB(.5,1000);
*/
    }


    public void remDist(double speed, int Target) {
        int dist = Target - RFPos;

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setTargetPosition(dist);
        robot.leftBack.setTargetPosition(dist);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);

        while(robot.rightFront.isBusy() && robot.leftBack.isBusy() && opModeIsActive()) {
        }

        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void turnAbsolute(int target) {
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
            telemetry.addData("accu", String.format("%03d", robot.zAccumulated));
            telemetry.update();
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        }

        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);     //turn off
        robot.leftBack.setPower(0);
    }

    public void turn(int degrees) {
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

    public void hinge(double speed, int target) {
        //RESET motor
        robot.hingeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.hingeMotor.setTargetPosition(-target);
        //run
        robot.hingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        robot.hingeMotor.setPower(speed);
        //update telemetry to show positions
        while (robot.hingeMotor.isBusy() && opModeIsActive()) {
        }
        //turn off power
        robot.hingeMotor.setPower(0);
        //turn off encoders
        robot.hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double speed, int Target) {
        //RESET encoders
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.rightFront.setTargetPosition(Target);
        robot.leftBack.setTargetPosition(Target);
        //run to position
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
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
        runtime.reset();
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


    public void driveRight(double speed, int Target) {
        //RESET encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.leftFront.setTargetPosition(Target);
        robot.rightBack.setTargetPosition(Target);
        //run to position
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
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
        //RESET encoders
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target
        robot.leftFront.setTargetPosition(-Target);
        robot.rightBack.setTargetPosition(-Target);
        //run to position
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
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

    public void driveStraightForward(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robot.leftBack.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
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
            telemetry.addData("3. Distance to go", duration + startPosition - robot.leftBack.getCurrentPosition());
            telemetry.update();
        }

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
}
