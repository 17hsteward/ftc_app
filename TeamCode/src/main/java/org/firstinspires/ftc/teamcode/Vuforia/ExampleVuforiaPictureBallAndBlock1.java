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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV Read Ball and Block", group = "Vuforia")
//@Disabled

public class ExampleVuforiaPictureBallAndBlock1 extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables roverRuckusImages;
    private VuforiaTrackable roverTarget;
    private VuforiaTrackable footprintTarget;
    private VuforiaTrackable cratersTarget;
    private VuforiaTrackable spaceTarget;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    Image RGBImage = null;
    Mat img = null;
    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    float robotX, robotY, robotZ, robotAngle;
    Boolean buttonPressed = false;
    private Boolean targetVisible = false;

    //Scaler values for HSV.  These work much better than BRG
    public final static Scalar yellowLow = new Scalar(0, 150, 150);  //
    public final static Scalar yellowHigh = new Scalar(60, 255, 255);
    public final static Scalar whiteLow = new Scalar(0, 0, 150);
    public final static Scalar whiteHigh = new Scalar(180, 10, 255);
    //Scaler values for BGR.  Not very good for yellow
    /*public final static Scalar yellowLow = new Scalar(0, 150, 150);  //
    public final static Scalar yellowHigh = new Scalar(100, 255, 255);
    public final static Scalar whiteLow = new Scalar(200, 100, 100);
    public final static Scalar whiteHigh = new Scalar(255, 255, 255);*/

    minerals mineralState;

    public enum minerals {
        MINERALS_NOT_VISIBLE,
        TWO_SILVER,
        SILVER_LEFT,
        SILVER_RIGHT,
    }


    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);// To add the camera view from the screen, add cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforiaLocalizer.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        roverRuckusImages = this.vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //Will track all 4 images

        // Setup the targets to be tracked
        roverTarget = roverRuckusImages.get(0);
        roverTarget.setName("Blue-Rover");
        roverTarget.setLocation(createMatrix(0, 1829, 152, 90, 0 ,0));

        footprintTarget = roverRuckusImages.get(1);
        footprintTarget.setName("Red-Footprint");
        footprintTarget.setLocation(createMatrix(0, -1829, 152, 90, 0 ,180));

        cratersTarget = roverRuckusImages.get(2);
        cratersTarget.setName("Front-Craters");
        cratersTarget.setLocation(createMatrix(-1829, 0, 152, 90, 0 ,90));

        spaceTarget = roverRuckusImages.get(3);
        spaceTarget.setName("Back-Space");
        spaceTarget.setLocation(createMatrix(1829, 0, 152, 90, 0, -90));

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(roverRuckusImages);

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, -90, 0, 0);

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        roverRuckusImages.activate();//Start tracking the data sets we care about.
        timer.reset();
    }


    @Override
    public void loop() {

        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

       try {
            vuforiaFrame = vuforiaLocalizer.getFrameQueue().take();
            RGBImage = getImageFromFrame(vuforiaFrame, PIXEL_FORMAT.RGB565);

            mineralState = getMineralState(RGBImage);

            vuforiaFrame.close();
        } catch (InterruptedException e) {
           e.printStackTrace();
       }

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

    //public minerals getMineralState(Image img) {
    public minerals getMineralState(Image img) {

        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());

        Mat mineralPic = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, mineralPic);

        Imgproc.cvtColor(mineralPic, mineralPic, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(mineralPic, mineralPic, new Size(9,9),2, 2 );

        Mat whiteMask = new Mat();
        Core.inRange(mineralPic, whiteLow, whiteHigh, whiteMask);
        Imgproc.GaussianBlur(whiteMask, whiteMask, new Size(9,9),2,2 );
        Moments whiteMmnts = Imgproc.moments(whiteMask, true);

        Bitmap bmOutWhite = Bitmap.createBitmap(whiteMask.width(), whiteMask.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(whiteMask, bmOutWhite);

        Mat yellowMask = new Mat();
        Core.inRange(mineralPic, yellowLow, yellowHigh, yellowMask);
        Imgproc.GaussianBlur(yellowMask, yellowMask, new Size(9,9),2, 2 );
        Moments yellowMmnts = Imgproc.moments(yellowMask, true);

        Bitmap bmOutYellow = Bitmap.createBitmap(yellowMask.width(), yellowMask.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(yellowMask, bmOutYellow);

        if (timer.seconds()> 2 && !buttonPressed) {
            buttonPressed = true;
            saveImage(bmOutWhite, "white");
            saveImage(bmOutYellow, "yellow");
        } /*else if (!gamepad1.x && buttonPressed) {
            buttonPressed = false;
        }*/


    return mineralState.MINERALS_NOT_VISIBLE;
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

    public void saveImage(Bitmap bitmap, String stringout) {
        if (bitmap != null) {
            String nameOfFile = "imageCaptured" + stringout;

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