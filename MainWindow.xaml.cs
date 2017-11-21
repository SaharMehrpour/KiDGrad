//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------


namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Collections;
    using System.Collections.Generic;

    using System.Windows.Controls;
    using System.Globalization;
    using System.Windows.Media.Imaging;

    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using System.Windows.Documents;
    using System.Text;
    using System.Windows.Threading;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    public partial class MainWindow : Window
    {
        ////====================================Added=====================================        
        public class KiDGraD
        {
            public static float scale = 55;//75
            public static Queue<Tuple<int, int>> rightHandGridInfo = new Queue<Tuple<int,int>>(); // It should have max size 8
            public static Queue<Tuple<int, int>> leftHandGridInfo = new Queue<Tuple<int, int>>();  // It should have max size 4
            public static int xIndex = (int)(MainWindow.RenderWidth / scale) + 2;
            public static int yIndex = (int)(MainWindow.RenderHeight / scale) + 2;
            public static bool[,] gridNode = new bool[xIndex, yIndex];
            public enum kidGradStateEnum
            {
                reset = 0,
                addNode = 1,
                removeNode = 2,
                addEdge = 3,
                removeEdge = 4
            }
            public static int kidGradState = 1;
            public static Tuple<int, int>[] endpointEdge = new Tuple<int, int>[2] { Tuple.Create(-1, -1), Tuple.Create(-1, -1) };

        }

        public class Graph
        {
            public static List<Node> nodes = new List<Node>();            
        }

        public class Node
        {
            public int xCoordinate;
            public int yCoordinate;
            public List<Node> adjacentNodes = new List<Node>();

            public Node(int x, int y)
            {
                xCoordinate = x;
                yCoordinate = y;
            }
        }

        private SpeechRecognitionEngine speechEngine;
        private const string MediumGreyBrushKey = "MediumGreyBrush";
        private static bool lockGestures = false;
        private bool brushForFlash = true; 

        private DispatcherTimer dt;      

        ////===============================End Added======================================

        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.SkyBlue, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            if (KinectSensor.KinectSensors.Count > 0)
            {
                sensor = KinectSensor.KinectSensors[0];
                ////======================================Added==================================================================
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                this.sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
                ////======================================End Added===============================================================
            }
        }
        ////======================================Added==================================================================
        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame cframe = e.OpenColorImageFrame()) 
            {
                if (cframe == null)
                    return;
                byte[] cbytes = new byte[cframe.PixelDataLength];
                cframe.CopyPixelDataTo(cbytes);
                int stride = cframe.Width * 4;
                Image1.Source = BitmapImage.Create(640, 480, 96, 96, PixelFormats.Bgr32, null, cbytes, stride);  
            }
        }
        ////======================================End Added===============================================================
        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {           
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();            

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                
                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                return;
                //this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            ////=============================ADDED FOR SPEECH==================================
            RecognizerInfo ri = GetKinectRecognizer();

            if (null != ri)
            {

                this.speechEngine = new SpeechRecognitionEngine(ri.Id);

                // Create a grammar from grammar definition XML file.
                using (var memoryStream = new MemoryStream(Encoding.ASCII.GetBytes(Properties.Resources.SpeechGrammar)))
                {
                    var g = new Grammar(memoryStream);
                    speechEngine.LoadGrammar(g);
                }

                speechEngine.SpeechRecognized += SpeechRecognized;

                // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                // This will prevent recognition accuracy from degrading over time.
                ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);

                speechEngine.SetInputToAudioStream(
                    sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
            else
            {
                return;
            }
            //=============================END OF ADDED FOR SPEECH===========================
            //==============================ADDED FOR FLASHING===============================
            dt = new DispatcherTimer();
            dt.Interval = new TimeSpan(0, 0, 1);
            dt.Tick += dispatcherTimer_Tick;
            dt.Start();
            //=======================END OF ADDED FOR FLASHING===============================
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                //dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, KiDGraD.scale, RenderHeight));
                ////======================================Added==================================================================
                DrawGrid(dc);
                DrawNodes(dc);
                DrawEdge(dc);
                ////==================================End of Added==============================================================
                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }

                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            ////======================================Added================================================================
            GestureDetection(skeleton, drawingContext);
            TrackLeftHand(skeleton);
            ////==================================End of Added==============================================================            

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints

            foreach (Joint joint in skeleton.Joints)
            {

                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }

        }
        ////================================Added===============================================
        public void DrawGrid(DrawingContext drawingContext)
        {
            int Height = (int)(MainWindow.RenderHeight / KiDGraD.scale);
            int Width = (int)(MainWindow.RenderWidth / KiDGraD.scale);
            Pen pen = new Pen(Brushes.SpringGreen, 3);
            for (int i = 1; i <= Height; i++) // horizontal lines
            {
                Point p = new Point(0, i * KiDGraD.scale);
                Point q = new Point(MainWindow.RenderWidth, i * KiDGraD.scale);
                drawingContext.DrawLine(pen, p, q);
            }
            for (int i = 1; i <= Width; i++) // vertical lines
            {
                Point p = new Point(i * KiDGraD.scale, 0);
                Point q = new Point(i * KiDGraD.scale, MainWindow.RenderHeight);
                drawingContext.DrawLine(pen, p, q);
            }
        }
        ////--------------------------------------------------------------------------------------       
        public void DrawNodes(DrawingContext drawingContext)
        {
            for (int i = 0; i < Graph.nodes.Count; i++)
            {
                Point center = new Point(Graph.nodes[i].xCoordinate * KiDGraD.scale, Graph.nodes[i].yCoordinate * KiDGraD.scale);
                Pen pen2 = new Pen(Brushes.Tomato, 2);
                Brush br = new SolidColorBrush(
                            System.Windows.Media.Color.FromRgb(255,0,0));
                br.Opacity = 1;
                drawingContext.DrawEllipse(br, pen2, center, 10, 10);

            }
            if (!KiDGraD.endpointEdge[0].Equals(Tuple.Create(-1, -1)))
            {
                if (brushForFlash)
                {
                    Point center = new Point(KiDGraD.endpointEdge[0].Item1 * KiDGraD.scale, KiDGraD.endpointEdge[0].Item2 * KiDGraD.scale);
                    Pen pen2 = new Pen(Brushes.Orange, 2);
                    Brush br = new SolidColorBrush(
                                System.Windows.Media.Color.FromRgb(255, 102, 0));
                    br.Opacity = 1;
                    drawingContext.DrawEllipse(br, pen2, center, 15, 15);
                }
            }
        }
        ////-------------------------------------------------------------------------------------- 
        public void DrawEdge(DrawingContext drawingContext)
        {
            for (int i = 0; i < Graph.nodes.Count; i++)
            {
                Point p = new Point(Graph.nodes[i].xCoordinate * KiDGraD.scale, Graph.nodes[i].yCoordinate * KiDGraD.scale);
                for (int j = 0; j < Graph.nodes[i].adjacentNodes.Count; j++)
                {
                    Point q = new Point(Graph.nodes[i].adjacentNodes[j].xCoordinate * KiDGraD.scale, Graph.nodes[i].adjacentNodes[j].yCoordinate * KiDGraD.scale);
                    Pen pen = new Pen(Brushes.Tomato, 5);
                    drawingContext.DrawLine(pen, p, q);
                }

            }
        }
        ////--------------------------------------------------------------------------------------
        private void TrackLeftHand(Skeleton skeleton)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandLeft].Position,
                DepthImageFormat.Resolution640x480Fps30);

            Point leftHand = new Point(depthPoint.X, depthPoint.Y);

            if (leftHand.X > KiDGraD.scale)
                return;
            
            Tuple<int, int> tuple = new Tuple<int, int>((int)(leftHand.X * 2 / KiDGraD.scale ), (int)(leftHand.Y * 2 / KiDGraD.scale ));
            Tuple<int, int>[] Array = KiDGraD.leftHandGridInfo.ToArray();

            if (KiDGraD.leftHandGridInfo.Count == 0)
                KiDGraD.leftHandGridInfo.Enqueue(tuple);
            else if (!tuple.Equals(Array[Array.Length - 1]))
            {
                KiDGraD.leftHandGridInfo.Enqueue(tuple); // save node by scale
                if (KiDGraD.leftHandGridInfo.Count > 4)
                    KiDGraD.leftHandGridInfo.Dequeue();
                if (KiDGraD.leftHandGridInfo.Count == 4)
                {
                    Tuple<int, int>[] leftHandGridInfo_Array = KiDGraD.leftHandGridInfo.ToArray();
                    if (leftHandGridInfo_Array[0].Item2 == leftHandGridInfo_Array[1].Item2) // in a same row
                    {
                        if (leftHandGridInfo_Array[1].Item1 == leftHandGridInfo_Array[2].Item1) // in a same column
                        {
                            if (leftHandGridInfo_Array[3].Item1 == leftHandGridInfo_Array[0].Item1)
                            {
                                if (leftHandGridInfo_Array[3].Item2 == leftHandGridInfo_Array[2].Item2)
                                {
                                    int nodeY = Math.Max(leftHandGridInfo_Array[1].Item2, leftHandGridInfo_Array[2].Item2);

                                    if (nodeY.Equals(3))
                                    {
                                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.addNode;
                                        currentCommand.Text = "ADD NODE";
                                        ReInitial();
                                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddNodeActive.gif", UriKind.Relative);
                                        AddNodeImage.Source = new BitmapImage(uriSource);
                                    }
                                    else if (nodeY.Equals(5))
                                    {
                                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.removeNode;
                                        currentCommand.Text = "REMOVE NODE";
                                        ReInitial();
                                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveNodeActive.gif", UriKind.Relative);
                                        RemoveNodeImage.Source = new BitmapImage(uriSource);
                                    }
                                    else if (nodeY.Equals(7))
                                    {
                                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.addEdge;
                                        currentCommand.Text = "ADD EDGE";
                                        ReInitial();
                                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddEdgeActive.gif", UriKind.Relative);
                                        AddEdgeImage.Source = new BitmapImage(uriSource);
                                    }
                                    else if (nodeY.Equals(9))
                                    {
                                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.removeEdge;
                                        currentCommand.Text = "REMOVE EDGE";
                                        ReInitial();
                                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveEdgeActive.gif", UriKind.Relative);
                                        RemoveEdgeImage.Source = new BitmapImage(uriSource);
                                    }
                                    else if (nodeY.Equals(1))
                                    {
                                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.reset;
                                        currentCommand.Text = "RESET";
                                        ReInitial();
                                        ResetKidGraD();
                                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/ResetActive.gif", UriKind.Relative);
                                        ResetImage.Source = new BitmapImage(uriSource);
                                    }
                                }

                            }
                        }
                    }
                }
            }
        }
        ////--------------------------------------------------------------------------------------
        private void GestureDetection(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (!lockGestures)
            {
                Point leftWrist = new Point(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y);
                Point rightWrist = new Point(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y);
                Point leftElbow = new Point(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y);
                Point rightElbow = new Point(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y);
                Point leftShoulder = new Point(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y);
                Point rightShoulder = new Point(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y);
                Point shoulderCentre = new Point(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y);

                double leftAngleElbow = Math.Atan2(leftElbow.Y - leftWrist.Y, leftElbow.X - leftWrist.X)
                    - Math.Atan2(leftElbow.Y - leftShoulder.Y, leftElbow.X - leftShoulder.X);
                if (leftAngleElbow < 0) leftAngleElbow += 2d * Math.PI;
                double rightAngleElbow = Math.Atan2(rightElbow.Y - rightWrist.Y, rightElbow.X - rightWrist.X)
                    - Math.Atan2(rightElbow.Y - rightShoulder.Y, rightElbow.X - rightShoulder.X);
                if (rightAngleElbow < 0) rightAngleElbow += 2d * Math.PI;
                double leftAngleShoulder = Math.Atan2(leftShoulder.Y - leftElbow.Y, leftShoulder.X - leftElbow.X)
                    - Math.Atan2(leftShoulder.Y - shoulderCentre.Y, leftShoulder.X - shoulderCentre.X);
                if (leftAngleShoulder < 0) leftAngleShoulder += 2d * Math.PI;
                double rightAngleShoulder = Math.Atan2(rightShoulder.Y - rightElbow.Y, rightShoulder.X - rightElbow.X)
                    - Math.Atan2(rightShoulder.Y - shoulderCentre.Y, rightShoulder.X - shoulderCentre.X);
                if (rightAngleShoulder < 0) rightAngleShoulder += 2d * Math.PI;
                double leftExtAngleShoulder = Math.Atan2(leftShoulder.Y - leftElbow.Y, leftShoulder.X - leftElbow.X);
                if (leftExtAngleShoulder < 0) leftExtAngleShoulder += 2d * Math.PI;
                double rightExtAngleShoulder = Math.Atan2(rightShoulder.Y - rightElbow.Y, rightShoulder.X - rightElbow.X);
                if (rightExtAngleShoulder < 0) rightExtAngleShoulder += 2d * Math.PI;
                double leftExtAngleElbow = Math.Atan2(leftWrist.Y - leftElbow.Y, leftWrist.X - leftElbow.X);
                if (leftExtAngleElbow < 0) leftExtAngleElbow += 2d * Math.PI;


                //if (leftExtAngleElbow * (180.0 / Math.PI) < 100d && leftExtAngleElbow * (180.0 / Math.PI) > 80) // '-|\
                //{
                //    KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.reset;
                //    currentCommand.Text = "RESET";
                //    ResetKidGraD();
                //}
                if (leftAngleShoulder * (180.0 / Math.PI) < 200d && leftAngleShoulder * (180.0 / Math.PI) > 160d // <|?
                    && leftAngleElbow * (180.0 / Math.PI) < 290d && leftAngleElbow * (180.0 / Math.PI) > 210d)
                {
                    ReInitial();
                    if (rightAngleShoulder * (180.0 / Math.PI) < 200d && rightAngleShoulder * (180.0 / Math.PI) > 160d // <|>
                    && rightAngleElbow * (180.0 / Math.PI) < 160d && rightAngleElbow * (180.0 / Math.PI) > 70d)
                    {
                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.addNode;
                        currentCommand.Text = "ADD NODE";
                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddNodeActive.gif", UriKind.Relative);
                        AddNodeImage.Source = new BitmapImage(uriSource);
                    }
                    else // <|\
                    {
                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.removeNode;
                        currentCommand.Text = "REMOVE NODE";
                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveNodeActive.gif", UriKind.Relative);
                        RemoveNodeImage.Source = new BitmapImage(uriSource);
                    }
                }
                else if ((leftExtAngleShoulder * (180.0 / Math.PI) < 20d || leftExtAngleShoulder * (180.0 / Math.PI) > 340) && // --|?
                    (leftAngleElbow * (180.0 / Math.PI) < 200d && leftAngleElbow * (180.0 / Math.PI) > 160d))
                {
                    ReInitial();
                    if ((rightExtAngleShoulder * (180.0 / Math.PI) < 210d && rightExtAngleShoulder * (180.0 / Math.PI) > 150d) && // --|--
                        (rightAngleElbow * (180.0 / Math.PI) < 200d && rightAngleElbow * (180.0 / Math.PI) > 160d))
                    {
                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.addEdge;
                        currentCommand.Text = "ADD EDGE";
                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddEdgeActive.gif", UriKind.Relative);
                        AddEdgeImage.Source = new BitmapImage(uriSource);

                    }
                    else // --|\
                    {
                        KiDGraD.kidGradState = (int)KiDGraD.kidGradStateEnum.removeEdge;
                        currentCommand.Text = "REMOVE EDGE";
                        var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveEdgeActive.gif", UriKind.Relative);
                        RemoveEdgeImage.Source = new BitmapImage(uriSource);
                    }
                }
            }
            TrackRightHand(skeleton);
        }
        ////-----------------------------------------------------------------
        public void TrackRightHand(Skeleton skeleton)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandRight].Position,
                   DepthImageFormat.Resolution640x480Fps30);
            
            Point rightHand = new Point(depthPoint.X, depthPoint.Y);

            Tuple<int, int> tuple = new Tuple<int, int>((int)(rightHand.X / KiDGraD.scale), (int)(rightHand.Y / KiDGraD.scale));                       
            Tuple<int, int>[] Array = KiDGraD.rightHandGridInfo.ToArray();
            
            if (KiDGraD.rightHandGridInfo.Count == 0)
                KiDGraD.rightHandGridInfo.Enqueue(tuple);
            else if (!tuple.Equals(Array[Array.Length-1]))
            {
                KiDGraD.rightHandGridInfo.Enqueue(tuple); // save node by scale
                if (KiDGraD.rightHandGridInfo.Count > 8)
                    KiDGraD.rightHandGridInfo.Dequeue();
                if (KiDGraD.rightHandGridInfo.Count == 8)
                {
                    Tuple<int, int>[] rightHandGridInfo_Array = KiDGraD.rightHandGridInfo.ToArray();
                    if (!rightHandGridInfo_Array[0].Equals(rightHandGridInfo_Array[4]) || !rightHandGridInfo_Array[1].Equals(rightHandGridInfo_Array[5])
                        || !rightHandGridInfo_Array[2].Equals(rightHandGridInfo_Array[6]) || !rightHandGridInfo_Array[3].Equals(rightHandGridInfo_Array[7]))
                        return;

                    if (rightHandGridInfo_Array[0].Item2 == rightHandGridInfo_Array[1].Item2) // in a same row
                    {
                        if (rightHandGridInfo_Array[1].Item1 == rightHandGridInfo_Array[2].Item1) // in a same column
                        {
                            if (rightHandGridInfo_Array[3].Item1 == rightHandGridInfo_Array[0].Item1)
                            {
                                if (rightHandGridInfo_Array[3].Item2 == rightHandGridInfo_Array[2].Item2)
                                {
                                    int nodeX = Math.Max(rightHandGridInfo_Array[0].Item1, rightHandGridInfo_Array[1].Item1);
                                    int nodeY = Math.Max(rightHandGridInfo_Array[1].Item2, rightHandGridInfo_Array[2].Item2);
                                    switch (KiDGraD.kidGradState)
                                    {
                                        case 1://(int)KiDGraD.kidGradStateEnum.addNode:
                                            AddNode(new Node(nodeX, nodeY));
                                            break;
                                        case 2://(int)KiDGraD.kidGradStateEnum.removeNode:
                                            RemoveNode(new Node(nodeX, nodeY));
                                            break;
                                        case 3://(int)KiDGraD.kidGradStateEnum.addEdge
                                            if (KiDGraD.gridNode[nodeX, nodeY])
                                            {
                                                if (KiDGraD.endpointEdge[0].Equals(Tuple.Create(-1, -1)))
                                                {
                                                    KiDGraD.endpointEdge[0] = new Tuple<int, int>(nodeX, nodeY);
                                                }
                                                else if (!KiDGraD.endpointEdge[0].Equals(new Tuple<int, int>(nodeX, nodeY)))
                                                {
                                                    KiDGraD.endpointEdge[1] = new Tuple<int, int>(nodeX, nodeY);
                                                    AddEdge();
                                                }
                                            }
                                            break;
                                        case 4://(int)KiDGraD.kidGradStateEnum.removeEdge
                                            if (KiDGraD.gridNode[nodeX, nodeY])
                                            {
                                                if (KiDGraD.endpointEdge[0].Equals(Tuple.Create(-1, -1)))
                                                {
                                                    KiDGraD.endpointEdge[0] = new Tuple<int, int>(nodeX, nodeY);
                                                }
                                                else if (!KiDGraD.endpointEdge[0].Equals(new Tuple<int, int>(nodeX, nodeY)))
                                                {
                                                    KiDGraD.endpointEdge[1] = new Tuple<int, int>(nodeX, nodeY);
                                                    RemoveEdge();
                                                }
                                            }
                                            break;
                                        default:
                                            break;
                                    }

                                }
                            }
                        }
                    }
                }
            }
        }
        ////-----------------------------------------------------------------
        public void AddNode(Node node)
        {
            if (!KiDGraD.gridNode[node.xCoordinate, node.yCoordinate])
            {
                Graph.nodes.Add(node);
                KiDGraD.gridNode[node.xCoordinate, node.yCoordinate] = true;
            }
        }
        ////-----------------------------------------------------------------
        public void ReInitial()
        {
            KiDGraD.endpointEdge[0] = Tuple.Create(-1, -1);
            KiDGraD.endpointEdge[1] = Tuple.Create(-1, -1);
            KiDGraD.rightHandGridInfo.Clear();
            KiDGraD.leftHandGridInfo.Clear();

            var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddNode.gif", UriKind.Relative);
            AddNodeImage.Source = new BitmapImage(uriSource);
            uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveNode.gif", UriKind.Relative);
            RemoveNodeImage.Source = new BitmapImage(uriSource);
            uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddEdge.gif", UriKind.Relative);
            AddEdgeImage.Source = new BitmapImage(uriSource);
            uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveEdge.gif", UriKind.Relative);
            RemoveEdgeImage.Source = new BitmapImage(uriSource);
            uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/Reset.gif", UriKind.Relative);
            ResetImage.Source = new BitmapImage(uriSource);
        }
        ////-----------------------------------------------------------------
        public void ResetKidGraD()
        {
            Graph.nodes.Clear();
            KiDGraD.gridNode = new bool[KiDGraD.xIndex, KiDGraD.yIndex];
            ReInitial();
            var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/ResetActive.gif", UriKind.Relative);
            ResetImage.Source = new BitmapImage(uriSource);
        }
        ////-----------------------------------------------------------------
        public void RemoveNode(Node node)
        {
            if (KiDGraD.gridNode[node.xCoordinate, node.yCoordinate])
            {
                KiDGraD.gridNode[node.xCoordinate, node.yCoordinate] = false;
                for (int k = 0; k < Graph.nodes.Count; k++)
                {
                    if (Graph.nodes[k].xCoordinate == node.xCoordinate && Graph.nodes[k].yCoordinate == node.yCoordinate)
                    {
                        Graph.nodes.RemoveAt(k);
                        // Remove adjacent vertices!!!
                        break;
                    }
                }
                for (int j = 0; j < Graph.nodes.Count; j++)
                {
                    for (int i = 0; i < Graph.nodes[j].adjacentNodes.Count; i++)
                    {
                        if (Graph.nodes[j].adjacentNodes[i].xCoordinate.Equals(node.xCoordinate)
                            && Graph.nodes[j].adjacentNodes[i].yCoordinate.Equals(node.yCoordinate))
                        {
                            Graph.nodes[j].adjacentNodes.RemoveAt(i);
                            continue;
                        }
                    }
                }
            }
        }
        ////-----------------------------------------------------------------
        public void AddEdge()
        {
            int k = 0;
            var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/AddEdgeActive.gif", UriKind.Relative);
            for (k = 0; k < Graph.nodes.Count; k++)
            {
                if (Graph.nodes[k].xCoordinate.Equals(KiDGraD.endpointEdge[0].Item1)
                    && Graph.nodes[k].yCoordinate.Equals(KiDGraD.endpointEdge[0].Item2))
                {
                    for (int i = 0; i < Graph.nodes[k].adjacentNodes.Count; i++)
                    {
                        if (Graph.nodes[k].adjacentNodes[i].xCoordinate.Equals(KiDGraD.endpointEdge[1].Item1)
                            && Graph.nodes[k].adjacentNodes[i].yCoordinate.Equals(KiDGraD.endpointEdge[1].Item2))
                        {
                            ReInitial();                            
                            AddEdgeImage.Source = new BitmapImage(uriSource);
                            return;
                        }
                    }
                    break;
                }
            }

            Graph.nodes[k].adjacentNodes.Add(new Node(KiDGraD.endpointEdge[1].Item1, KiDGraD.endpointEdge[1].Item2));
            for (int j = 0; j < Graph.nodes.Count; j++)
            {
                if (Graph.nodes[j].xCoordinate.Equals(KiDGraD.endpointEdge[1].Item1)
                    && Graph.nodes[j].yCoordinate.Equals(KiDGraD.endpointEdge[1].Item2))
                {
                    Graph.nodes[j].adjacentNodes.Add(new Node((int)KiDGraD.endpointEdge[0].Item1, (int)KiDGraD.endpointEdge[0].Item2));
                    break;
                }
            }
            ReInitial();
            AddEdgeImage.Source = new BitmapImage(uriSource);
        }
        ////-----------------------------------------------------------------
        public void RemoveEdge()
        {
            int flag = 0;
            for (int k = 0; k < Graph.nodes.Count; k++)
            {
                if (Graph.nodes[k].xCoordinate.Equals(KiDGraD.endpointEdge[0].Item1)
                    && Graph.nodes[k].yCoordinate.Equals(KiDGraD.endpointEdge[0].Item2))
                {
                    for (int i = 0; i < Graph.nodes[k].adjacentNodes.Count; i++)
                    {
                        if (Graph.nodes[k].adjacentNodes[i].xCoordinate.Equals(KiDGraD.endpointEdge[1].Item1)
                            && Graph.nodes[k].adjacentNodes[i].yCoordinate.Equals(KiDGraD.endpointEdge[1].Item2))
                        {
                            Graph.nodes[k].adjacentNodes.RemoveAt(i);
                            flag++;
                        }
                    }
                }
                if (Graph.nodes[k].xCoordinate.Equals(KiDGraD.endpointEdge[1].Item1)
                    && Graph.nodes[k].yCoordinate.Equals(KiDGraD.endpointEdge[1].Item2))
                {
                    for (int j = 0; j < Graph.nodes[k].adjacentNodes.Count; j++)
                    {
                        if (Graph.nodes[k].adjacentNodes[j].xCoordinate.Equals(KiDGraD.endpointEdge[0].Item1)
                            && Graph.nodes[k].adjacentNodes[j].yCoordinate.Equals(KiDGraD.endpointEdge[0].Item2))
                        {
                            Graph.nodes[k].adjacentNodes.RemoveAt(j);
                            flag++;
                        }
                    }
                }
            }
            if (flag == 2)
            {
                ReInitial();
                var uriSource = new Uri(@"/SkeletonBasics-WPF;component/Images/RemoveEdgeActive.gif", UriKind.Relative);
                RemoveEdgeImage.Source = new BitmapImage(uriSource);
            }
        }
        ////-----------------------------------------------------------------
        private Node EstimateRightHandScale(Skeleton skeleton)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandRight].Position,
                    DepthImageFormat.Resolution640x480Fps30);
            
            Point rightHand = new Point(depthPoint.X, depthPoint.Y);

            int pointX = (int)((rightHand.X / KiDGraD.scale) * KiDGraD.scale); // Rounding pixel
            if (Math.Abs(rightHand.X - pointX) > KiDGraD.scale/2)
                pointX = Math.Min((int)(MainWindow.RenderWidth / KiDGraD.scale) + 1, pointX + (int)KiDGraD.scale);

            int pointY = (int)((rightHand.Y / KiDGraD.scale) * KiDGraD.scale); // Rounding pixel
            if (Math.Abs(rightHand.Y - pointY) > KiDGraD.scale/2)
                pointY = Math.Min((int)(MainWindow.RenderHeight / KiDGraD.scale) + 1, pointY + (int)KiDGraD.scale);
            return new Node(pointX / (int)KiDGraD.scale, pointY / (int)KiDGraD.scale); // save node by scale
        }
        ////-----------------------------------------------------------------
        private Node EstimateLeftHandScale(Skeleton skeleton)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandLeft].Position,
                    DepthImageFormat.Resolution640x480Fps30);
            
            Point rightHand = new Point(depthPoint.X, depthPoint.Y);

            int pointX = (int)((rightHand.X / KiDGraD.scale) * KiDGraD.scale); // Rounding pixel
            if (Math.Abs(rightHand.X - pointX) > KiDGraD.scale/2)
                pointX = pointX + (int) KiDGraD.scale;

            int pointY = (int)((rightHand.Y / KiDGraD.scale) * KiDGraD.scale); // Rounding pixel
            if (Math.Abs(rightHand.Y - pointY) > KiDGraD.scale/2)
                pointY = pointY + (int)KiDGraD.scale;
            return new Node(pointX / (int)KiDGraD.scale, pointY / (int)KiDGraD.scale); // save node by scale
        }
        ////=========================End===Added===============================================

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            //DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));


        }        

        ////==============================ADD SPEECH=======================

        private static RecognizerInfo GetKinectRecognizer()
        {
            foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }
        ////------------------------------------------------------
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "LOCK":
                        this.currentCommand.Foreground = Brushes.Red;
                        lockGestures = true;
                        break;

                    case "UNLOCK":
                        this.currentCommand.Foreground = Brushes.Black;
                        lockGestures = false;
                        break;
                }
            }
        }
        ////===============================END ADD SPEECH====================
        ////==============================ADDED FOR FLASHING===============================
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            if (brushForFlash)
                brushForFlash = false;
            else
                brushForFlash = true;            
        }
        ////=======================END OF ADDED FOR FLASHING===============================
    }
}