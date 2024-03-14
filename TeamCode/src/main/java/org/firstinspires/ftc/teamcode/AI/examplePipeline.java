package org.firstinspires.ftc.teamcode.AI;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static java.lang.Math.abs;

import android.graphics.Camera;

import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class examplePipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public examplePipeline(Telemetry t) { telemetry = t; };

    public enum Location {
        LEFT,
        RIGHT,

        CENTER,
        NOT_FOUND
    }

    private Location location;

    public Mat matRed;
    public Mat matBlue;

    static final Rect left = new Rect(
            new Point(60, 35),
            new Point(120, 75)
    );

    static final Rect center = new Rect(
            new Point(140, 35),
            new Point(250, 75)
    );

    static final Rect right = new Rect(
            new Point(350, 35),
            new Point(450, 75)
    );

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,matRed,Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input,matBlue,Imgproc.COLOR_RGB2HSV);

        Scalar lowValueForRed = new Scalar(0, 70, 50);
        Scalar highValueRed = new Scalar(0, 255, 255);

        Scalar lowValueForBlue = new Scalar(90, 50, 50);
        Scalar highValueBlue = new Scalar(130, 255, 255);

        Core.inRange(matRed, lowValueForRed, highValueRed, matRed);
        Core.inRange(matBlue, lowValueForBlue, highValueBlue, matBlue);

        Mat blue_left = matBlue.submat(left);
        Mat blue_center = matBlue.submat(center);
        Mat blue_right = matBlue.submat(right);

        Mat red_left = matBlue.submat(left);
        Mat red_center = matBlue.submat(center);
        Mat red_right = matBlue.submat(right);

        double left_blue_value = Core.sumElems(blue_left).val[0] / left.area() / 255;
        double center_blue_value = Core.sumElems(blue_center).val[0] / center.area() / 255;
        double right_blue_value = Core.sumElems(blue_right).val[0] / right.area() / 255;

        double left_red_value = Core.sumElems(red_left).val[0] / left.area() / 255;
        double center_red_value = Core.sumElems(red_center).val[0] / center.area() / 255;
        double right_red_value = Core.sumElems(red_right).val[0] / right.area() / 255;

        blue_left.release();
        blue_center.release();
        blue_right.release();

        red_left.release();
        red_center.release();
        red_right.release();

        telemetry.addData("Blue left raw value", (int)Core.sumElems(blue_left).val[0]);
        telemetry.addData("Blue center raw value", (int)Core.sumElems(blue_left).val[0]);
        telemetry.addData("Blue right raw value", (int)Core.sumElems(blue_left).val[0]);

        telemetry.addData("red left raw value", (int)Core.sumElems(red_left).val[0]);
        telemetry.addData("red center raw value", (int)Core.sumElems(red_center).val[0]);
        telemetry.addData("red right raw value", (int)Core.sumElems(red_right).val[0]);

        telemetry.addData("Blue picture width: ", matBlue.width());
        telemetry.addData("Blue picture Height: ", matBlue.height());

        telemetry.addData("Red picture width: ", matRed.width());
        telemetry.addData("Red picture Height: ", matRed.height());

        telemetry.update();

        if(left_blue_value > left_red_value || center_blue_value > center_red_value || right_blue_value > right_red_value){

            if (left_blue_value > center_blue_value && left_blue_value > right_blue_value){
                location = Location.LEFT;
            }else if (left_blue_value < center_blue_value && center_blue_value > right_blue_value){
                location = Location.CENTER;
            }else if (right_blue_value > center_blue_value && left_blue_value < right_blue_value){
                location = Location.RIGHT;
            }

            return matBlue;
        }else {
            if (left_red_value > center_red_value && left_red_value > right_red_value){
                location = Location.LEFT;
            }else if (left_red_value < center_red_value && center_red_value > right_red_value){
                location = Location.CENTER;
            }else if (right_red_value > center_red_value && left_red_value < right_red_value){
                location = Location.RIGHT;
            }

            return matRed;
        }
    }

    public Location getLocation(){
        return location;
    }
}
