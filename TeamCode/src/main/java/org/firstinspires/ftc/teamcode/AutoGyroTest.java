package org.firstinspires.ftc.teamcode;

/*
Modern Robotics Gyro Straight Example
Updated 11/3/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Gyro Sensor named "gyro"
Motor named "ml" for left drive train
Motor named "mr" for right drive train

For more information, go to http://modernroboticsedu.com/course/view.php?id=4
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Straight", group = "4719")
@Disabled
public class AutoGyroTest extends LinearOpMode {  //Linear op mode is being used so the program does not get stuck in loop()
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftBack;  //Left Drive Motor
    DcMotor rightFront;  //Right Drive Motor
    DcMotor rightBack;
    DcMotor leftFront;

    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftBack = hardwareMap.dcMotor.get("leftBack");  //Config File
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack =hardwareMap.dcMotor.get("rightBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);  //This robot has two gears between motors and wheels. If your robot does not, you will need to reverse only the opposite motor
        leftFront.setDirection(DcMotor.Direction.REVERSE);  //This robot has two gears between motors and wheels. If your robot does not, you will need to reverse only the opposite motor

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        waitForStart();
        runtime.reset();

        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            driveStraightForward(30000, 0.3); //Drive straight for 30,000 encoder ticks (basically forever)
        }
    }


    public void driveStraightForward(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;


        double target = mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = leftBack.getCurrentPosition();  //Starting position

        while (leftBack.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            zAccumulated = mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftFSpeed = 0 + (zAccumulated-target)/ 100;
            rightBSpeed = 0 - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation



            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftBack.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            rightBack.setPower(rightBSpeed);
            leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", leftBack.getPower());
            telemetry.addData("2. Right", rightFront.getPower());
            telemetry.addData("3. LeftF", leftFront.getPower());
            telemetry.addData("4. RightB", rightBack.getPower());
            telemetry.addData("3. Distance to go", duration + startPosition - leftBack.getCurrentPosition());
            telemetry.update();
        }

        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                leftBack.setPower(turnSpeed);
                rightFront.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                leftBack.setPower(-turnSpeed);
                rightFront.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        leftBack.setPower(0);  //Stop the motors
        rightFront.setPower(0);

    }

}
