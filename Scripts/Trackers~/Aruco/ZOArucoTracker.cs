using System.Collections;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ArucoModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UtilsModule;


namespace ZO.Trackers {
    public class ZOArucoTracker : MonoBehaviour {

        public struct ZOArucoTrackerDetection {
            public int arucoId;
            public Matrix4x4 transform;
        }
        public ArUcoDictionary dictionaryId = ArUcoDictionary.DICT_4X4_50;

        /// <summary>
        /// The length of the markers' side. Normally, unit is meters.
        /// </summary>
        public float _markerLengthMeters = 0.1f;
        public ZO.Sensors.ZORGBCamera _rgbCamera;  // TODO: make this more generic, like an interface class
        public bool _debug = true;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// this, Detected Markers
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<ZOArucoTracker, List<ZOArucoTrackerDetection>, Task> OnPublishDelegate { get; set; }


        private Texture2D _debugTexture = null;
        private Mat _rgbMat = null;

        // Start is called before the first frame update
        void Start() {
            _rgbCamera.OnPublishRGBImageDelegate = OnRGBCameraUpdate;
        }

        // Update is called once per frame
        void Update() {

        }

        private void OnGUI() {
            if (_debug == true) {

                if (_rgbMat != null && _debugTexture == null) {
                    _debugTexture = new Texture2D(_rgbMat.cols(), _rgbMat.rows(), TextureFormat.RGBA32, false);
                }
                if (_rgbMat != null && _debugTexture != null) {
                    Utils.matToTexture2D(_rgbMat, _debugTexture, false, 0);
                }

                if (_debugTexture) {
                    GUI.DrawTexture(new UnityEngine.Rect(20, 320, _debugTexture.width, _debugTexture.height), _debugTexture, ScaleMode.ScaleToFit);
                }
            }
        }

        private Task OnRGBCameraUpdate(ZO.Sensors.ZORGBCamera camera, string cameraId, int width, int height, byte[] rgb24) {

            Mat rgbMat = new Mat(height, width, CvType.CV_8UC3);
            MatUtils.copyToMat<byte>(rgb24, rgbMat);
            Core.flip(rgbMat, rgbMat, 1);  //IMPORTANT OR DETECTION WILL NOT WORK!!!!

            List<ZOArucoTrackerDetection> detectedMarkers = DetectMarkers(rgbMat);

            if (_debug == true) {
                if (_rgbMat == null) {
                    _rgbMat = new Mat(height, width, CvType.CV_8UC3);
                }
                rgbMat.copyTo(_rgbMat);

            }

            if (OnPublishDelegate!=null) {
                OnPublishDelegate(this, new List<ZOArucoTrackerDetection>(detectedMarkers));
            } 

            return Task.CompletedTask;
        }

        public List<ZOArucoTrackerDetection> DetectMarkers(Mat rgbMat) {

            List<ZOArucoTrackerDetection> results = new List<ZOArucoTrackerDetection>();

            // Debug.Log("imgMat dst ToString " + rgbMat.ToString());

            float width = rgbMat.width();
            float height = rgbMat.height();
            float imageSizeScale = 1.0f;
            float widthScale = (float)Screen.width / width;
            float heightScale = (float)Screen.height / height;
            if (widthScale < heightScale) {
                // Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
                imageSizeScale = (float)Screen.height / (float)Screen.width;
            } else {
                // Camera.main.orthographicSize = height / 2;
            }

            // set camera parameters.
            int max_d = (int)Mathf.Max(width, height);
            double fx = max_d;
            double fy = max_d;
            double cx = width / 2.0f;
            double cy = height / 2.0f;
            Mat camMatrix = new Mat(3, 3, CvType.CV_64FC1);
            camMatrix.put(0, 0, fx);
            camMatrix.put(0, 1, 0);
            camMatrix.put(0, 2, cx);
            camMatrix.put(1, 0, 0);
            camMatrix.put(1, 1, fy);
            camMatrix.put(1, 2, cy);
            camMatrix.put(2, 0, 0);
            camMatrix.put(2, 1, 0);
            camMatrix.put(2, 2, 1.0f);
            // Debug.Log("camMatrix " + camMatrix.dump());


            MatOfDouble distCoeffs = new MatOfDouble(0, 0, 0, 0);
            // Debug.Log("distCoeffs " + distCoeffs.dump());


            // calibration camera matrix values.
            Size imageSize = new Size(width * imageSizeScale, height * imageSizeScale);
            double apertureWidth = 0;
            double apertureHeight = 0;
            double[] fovx = new double[1];
            double[] fovy = new double[1];
            double[] focalLength = new double[1];
            Point principalPoint = new Point(0, 0);
            double[] aspectratio = new double[1];

            Calib3d.calibrationMatrixValues(camMatrix, imageSize, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectratio);

            // Debug.Log("imageSize " + imageSize.ToString());
            // Debug.Log("apertureWidth " + apertureWidth);
            // Debug.Log("apertureHeight " + apertureHeight);
            // Debug.Log("fovx " + fovx[0]);
            // Debug.Log("fovy " + fovy[0]);
            // Debug.Log("focalLength " + focalLength[0]);
            // Debug.Log("principalPoint " + principalPoint.ToString());
            // Debug.Log("aspectratio " + aspectratio[0]);


            // To convert the difference of the FOV value of the OpenCV and Unity. 
            double fovXScale = (2.0 * Mathf.Atan((float)(imageSize.width / (2.0 * fx)))) / (Mathf.Atan2((float)cx, (float)fx) + Mathf.Atan2((float)(imageSize.width - cx), (float)fx));
            double fovYScale = (2.0 * Mathf.Atan((float)(imageSize.height / (2.0 * fy)))) / (Mathf.Atan2((float)cy, (float)fy) + Mathf.Atan2((float)(imageSize.height - cy), (float)fy));

            // Debug.Log("fovXScale " + fovXScale);
            // Debug.Log("fovYScale " + fovYScale);
            Mat ids = new Mat();
            List<Mat> corners = new List<Mat>();
            List<Mat> rejectedCorners = new List<Mat>();
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Mat rotMat = new Mat(3, 3, CvType.CV_64FC1);

            DetectorParameters detectorParams = DetectorParameters.create();
            Dictionary dictionary = Aruco.getPredefinedDictionary((int)dictionaryId);


            // detect markers.
            Aruco.detectMarkers(rgbMat, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);

            // Debug.Log("INFO: Number of markers detected: " + ids.total());
            // if at least one marker detected
            if (ids.total() > 0) {
                if (_debug) {
                    Aruco.drawDetectedMarkers(rgbMat, corners, ids, new Scalar(0, 255, 0));
                }
                

                // estimate pose.
                Aruco.estimatePoseSingleMarkers(corners, _markerLengthMeters, camMatrix, distCoeffs, rvecs, tvecs);

                for (int i = 0; i < ids.total(); i++) {


                    // Get translation vector
                    double[] tvecArr = tvecs.get(i, 0);

                    // Get rotation vector
                    double[] rvecArr = rvecs.get(i, 0);
                    Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
                    rvec.put(0, 0, rvecArr);

                    // Convert rotation vector to rotation matrix.
                    Calib3d.Rodrigues(rvec, rotMat);
                    double[] rotMatArr = new double[rotMat.total()];
                    rotMat.get(0, 0, rotMatArr);

                    // Convert OpenCV camera extrinsic parameters to Unity Matrix4x4.
                    Matrix4x4 transformationM = new Matrix4x4(); // from OpenCV
                    transformationM.SetRow(0, new Vector4((float)rotMatArr[0], (float)rotMatArr[1], (float)rotMatArr[2], (float)tvecArr[0]));
                    transformationM.SetRow(1, new Vector4((float)rotMatArr[3], (float)rotMatArr[4], (float)rotMatArr[5], (float)tvecArr[1]));
                    transformationM.SetRow(2, new Vector4((float)rotMatArr[6], (float)rotMatArr[7], (float)rotMatArr[8], (float)tvecArr[2]));
                    transformationM.SetRow(3, new Vector4(0, 0, 0, 1));
                    // Debug.Log("transformationM " + transformationM.ToString());

                    ZOArucoTrackerDetection detection = new ZOArucoTrackerDetection();
                    int [] currentId = new int[1];
                    // ids.get(0, i, currentId);
                    ids.get(i, 0, currentId);
                    detection.arucoId = currentId[0];
                    detection.transform = transformationM;
                    results.Add(detection);

                }
            }

            return results;

        }
        public List<ZOArucoTrackerDetection> DetectMarkers(Texture2D inputImageTexture) {

            int width = inputImageTexture.width;
            int height = inputImageTexture.height;

            Mat rgbMat = new Mat(height, width, CvType.CV_8UC3);
            Utils.texture2DToMat(inputImageTexture, rgbMat);

            return DetectMarkers(rgbMat);
        }

        public enum ArUcoDictionary {
            DICT_4X4_50 = Aruco.DICT_4X4_50,
            DICT_4X4_100 = Aruco.DICT_4X4_100,
            DICT_4X4_250 = Aruco.DICT_4X4_250,
            DICT_4X4_1000 = Aruco.DICT_4X4_1000,
            DICT_5X5_50 = Aruco.DICT_5X5_50,
            DICT_5X5_100 = Aruco.DICT_5X5_100,
            DICT_5X5_250 = Aruco.DICT_5X5_250,
            DICT_5X5_1000 = Aruco.DICT_5X5_1000,
            DICT_6X6_50 = Aruco.DICT_6X6_50,
            DICT_6X6_100 = Aruco.DICT_6X6_100,
            DICT_6X6_250 = Aruco.DICT_6X6_250,
            DICT_6X6_1000 = Aruco.DICT_6X6_1000,
            DICT_7X7_50 = Aruco.DICT_7X7_50,
            DICT_7X7_100 = Aruco.DICT_7X7_100,
            DICT_7X7_250 = Aruco.DICT_7X7_250,
            DICT_7X7_1000 = Aruco.DICT_7X7_1000,
            DICT_ARUCO_ORIGINAL = Aruco.DICT_ARUCO_ORIGINAL,
        }

    }
}