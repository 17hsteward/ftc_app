/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.Vuforia;


import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.CameraField;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

@Autonomous(name = "OpenCV Read Color", group = "Vuforia")
//@Disabled

public class ExampleVuforiaGrabPictureAndReadColor extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables velocityVortexImages;
    private VuforiaTrackable wheelsTarget;
    private VuforiaTrackable toolsTarget;
    private VuforiaTrackable legosTarget;
    private VuforiaTrackable gearsTarget;
    private VuforiaTrackableDefaultListener wheelsListener;
    private VuforiaTrackableDefaultListener toolsListener;
    private VuforiaTrackableDefaultListener legosListener;
    private VuforiaTrackableDefaultListener gearsListener;

    Image RGBImage = null;
    Mat img = null;
    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix robotLocationTransform;

    float robotX, robotY, robotZ, robotAngle;
    Boolean buttonPressed = false;

    public final static Scalar blueLow = new Scalar(110, 0, 100);
    public final static Scalar blueHigh = new Scalar(130, 255, 255);

    beacon beaconState;

    public enum beacon {
        BEACON_NOT_VISIBLE,
        BEACON_RED_BLUE,
        BEACON_BLUE_RED,
        BEACON_ALL_RED,
        BEACON_ALL_BLUE
    }


    @Override
    public void init() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforiaLocalizer.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        velocityVortexImages = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //Will track all 4 images

        // Setup the targets to be tracked
        wheelsTarget = velocityVortexImages.get(0);
        wheelsTarget.setName("Wheels");
        wheelsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        toolsTarget = velocityVortexImages.get(1);
        toolsTarget.setName("Tools");
        toolsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        legosTarget = velocityVortexImages.get(2);
        legosTarget.setName("Legos");
        legosTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        gearsTarget = velocityVortexImages.get(3);
        gearsTarget.setName("Gears");
        gearsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0, 90));


        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 180);

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        wheelsListener = (VuforiaTrackableDefaultListener) wheelsTarget.getListener();
        wheelsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        toolsListener = (VuforiaTrackableDefaultListener) toolsTarget.getListener();
        toolsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        legosListener = (VuforiaTrackableDefaultListener) legosTarget.getListener();
        legosListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        gearsListener = (VuforiaTrackableDefaultListener) gearsTarget.getListener();
        gearsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);


        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
        beaconState = beacon.BEACON_NOT_VISIBLE;
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        velocityVortexImages.activate();//Start tracking the data sets we care about.
        robotLocationTransform = wheelsListener.getUpdatedRobotLocation();
    }


    @Override
    public void loop() {

        if (wheelsListener.isVisible()) {
            robotLocationTransform = wheelsListener.getUpdatedRobotLocation();
        } else if (toolsListener.isVisible()) {
            robotLocationTransform = toolsListener.getUpdatedRobotLocation();
        } else if (legosListener.isVisible()) {
            robotLocationTransform = legosListener.getUpdatedRobotLocation();
        } else if (gearsListener.isVisible()) {
            robotLocationTransform = gearsListener.getUpdatedRobotLocation();
        }

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;

            VectorF trans = robotLocationTransform.getTranslation();

            robotX = trans.get(0);
            robotY = trans.get(1);
            robotZ = trans.get(2);
            robotAngle = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;


            try {
                vuforiaFrame = vuforiaLocalizer.getFrameQueue().take();
                RGBImage = getImageFromFrame(vuforiaFrame, PIXEL_FORMAT.RGB565);
                beaconState = getBeaconState(RGBImage, legosListener, vuforiaLocalizer.getCameraCalibration());

                vuforiaFrame.close();

            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("Robot Heading", robotAngle);
        telemetry.addData("RobotX", robotX);
        telemetry.addData("RobotY", robotY);
        telemetry.addData(wheelsTarget.getName(), wheelsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(toolsTarget.getName(), toolsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(legosTarget.getName(), legosListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(gearsTarget.getName(), gearsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData("Pos", formatMatrix(lastLocation));
        telemetry.addData("Beacon State ", beaconState.toString());
    }


    @Override
    public void stop() {

    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public beacon getBeaconState(Image img, VuforiaTrackableDefaultListener target, CameraCalibration camCal) {
        OpenGLMatrix pose = target.getPose();

        if (pose != null && img != null && img.getPixels() != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-120, 270, 0)).getData();
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, 270, 0)).getData();
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, 140, 0)).getData();
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-120, 140, 0)).getData();

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[0][0],corners[1][0]), Math.min(corners[2][0],corners[3][0]));
            float y = Math.min(Math.min(corners[0][1],corners[1][1]), Math.min(corners[2][1],corners[3][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols()) ? crop.cols() - x : width;
            height = (y + height > crop.rows()) ? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            Bitmap bmOut = Bitmap.createBitmap((int) width, (int) height, Bitmap.Config.RGB_565);
            Utils.matToBitmap(mask, bmOut);

            if (gamepad1.x && !buttonPressed) {
                buttonPressed = true;
                saveImage(bmOut);
            } else if (!gamepad1.x && buttonPressed) {
                buttonPressed = false;
            }

            if (mmnts.get_m00() / mask.total() > 0.8) {
                return beacon.BEACON_ALL_BLUE;
            } else if (mmnts.get_m00() / mask.total() < 0.1) {
                return beacon.BEACON_ALL_RED;
            }

            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows()/2) {
                return beacon.BEACON_RED_BLUE;
            } else {
                return beacon.BEACON_BLUE_RED;
            }
        }

        return beacon.BEACON_NOT_VISIBLE;
    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {

        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if(frame.getImage(i).getFormat() == pixelFormat){
                return frame.getImage(i);
            }
        }

        return null;
    }

    public void saveImage(Bitmap bitmap) {
        if (bitmap != null) {
            String nameOfFile = "imageCaptured";

            String state = Environment.getExternalStorageState();
            if (Environment.MEDIA_MOUNTED.equals(state)) {
                Log.i("Mike", "Able to write to storage");
            } else {
                Log.i("Mike", "Cannot write to storage");
            }


            File dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);

            Log.i("Mike", "saveImage: " + dir.getPath());

            if (!dir.exists()) {
                Log.i("Mike", "Dir does not exist");
            } else {
                Log.i("Mike", "Dir Exists");

                File file = new File(dir, nameOfFile + ".jpg");

                try {
                    FileOutputStream fOut = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fOut);

                    fOut.flush();
                    fOut.close();

                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                    Log.i("Mike", e.toString());
                }
            }
        }
    }


}