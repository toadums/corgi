using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit.Interaction;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Corgie
{
    public class Kinect
    {

        bool NearMode = false;

        KinectSensor Sensor;
        Skeleton[] _skeletonData;

        Skeleton _skeleton = null;
        SpeechRecognitionEngine SpeechEngine;

        Corgi2 WorldSize;

        InteractionStream _interactionStream;
        private UserInfo[] _userInfos; //the information about the interactive users
        Skeleton PlayerSkeleton
        {
            get
            {
                if (Length(_skeleton.Position) > 0)
                    return _skeleton;
                else
                    return null;
            }

            set
            {
                _skeleton = value;
            }
        }

        private static string _col = "";

        private Corgi2 _hand;

        public Corgi2 Hand
        {
            get
            {
                return _hand;
            }
        }

        public String LastColor
        {
            get
            {
                string col = _col;
                _col = "";
                return col;
            }

            set
            {
                _col = value;
            }
        }

        public Kinect(int x, int y)
        {
            WorldSize = new Corgi2(x, y);
        }

        public void Init()
        {

            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    Sensor = potentialSensor;
                    break;
                }
            }

            if (Sensor != null)
            {

                _skeleton = new Skeleton();
                _userInfos = new UserInfo[InteractionFrame.UserInfoArrayLength];
                _skeletonData = new Skeleton[Sensor.SkeletonStream.FrameSkeletonArrayLength];

                Sensor.DepthStream.Range = DepthRange.Default;
                Sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                Sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                Sensor.SkeletonStream.EnableTrackingInNearRange = true;
                Sensor.SkeletonStream.Enable(new TransformSmoothParameters()
                {
                    Smoothing = 0.5f,
                    Correction = 0.5f,
                    Prediction = 0.5f,
                    JitterRadius = 0.05f,
                    MaxDeviationRadius = 0.04f
                });

                _interactionStream = new InteractionStream(Sensor, new DummyInteractionClient());
                _interactionStream.InteractionFrameReady += InteractionStreamOnInteractionFrameReady;

                Sensor.DepthFrameReady += SensorOnDepthFrameReady;
                Sensor.SkeletonFrameReady += Sensor_SkeletonFrameReady;

                try
                {
                    Sensor.Start();
                }
                catch (Exception e)
                {
                    Sensor = null;
                }


                #region INITSPEECH
                RecognizerInfo ri = GetKinectRecognizer();

                if (null != ri && Sensor != null)
                {
                    SpeechEngine = new SpeechRecognitionEngine(ri.Id);


                    /****************************************************************
                    * 
                    * Use this code to create grammar programmatically rather than from
                    * a grammar file.
                    */
                    var directions = new Choices();
                    directions.Add(new SemanticResultValue("yellow", "YELLOW"));
                    directions.Add(new SemanticResultValue("yeller", "YELLOW"));
                    directions.Add(new SemanticResultValue("red", "RED"));
                    directions.Add(new SemanticResultValue("read", "RED"));
                    directions.Add(new SemanticResultValue("blue", "BLUE"));
                    directions.Add(new SemanticResultValue("blew", "BLUE"));
                    directions.Add(new SemanticResultValue("green", "GREEN"));
                    directions.Add(new SemanticResultValue("grin", "GREEN"));
                    directions.Add(new SemanticResultValue("sit", "SIT"));
                    directions.Add(new SemanticResultValue("stand", "STAND"));


                    var gb = new GrammarBuilder { Culture = ri.Culture };
                    gb.Append(directions);

                    var g = new Grammar(gb);
                    SpeechEngine.LoadGrammar(g);



                    SpeechEngine.SpeechRecognized += SpeechRecognized;

                    // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                    // This will prevent recognition accuracy from degrading over time.
                    ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);

                    SpeechEngine.SetInputToAudioStream(
                        Sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                    SpeechEngine.RecognizeAsync(RecognizeMode.Multiple);
                }
                #endregion
            }
        }

        private void SensorOnDepthFrameReady(object sender, DepthImageFrameReadyEventArgs depthImageFrameReadyEventArgs)
        {
            using (DepthImageFrame depthFrame = depthImageFrameReadyEventArgs.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                    return;

                try
                {
                    _interactionStream.ProcessDepth(depthFrame.GetRawPixelData(), depthFrame.Timestamp);
                }
                catch (InvalidOperationException)
                {
                    // DepthFrame functions may throw when the sensor gets
                    // into a bad state.  Ignore the frame in that case.
                }
            }
        }

        #region SKELETON
        private void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skelFrame = e.OpenSkeletonFrame())
            {
                if (skelFrame != null)
                {
                    skelFrame.CopySkeletonDataTo(_skeletonData);
                    var accelerometerReading = Sensor.AccelerometerGetCurrentReading();
                    _interactionStream.ProcessSkeleton(_skeletonData, accelerometerReading, skelFrame.Timestamp);
               
                }
            }

            foreach (Skeleton s in _skeletonData)
            {
                if (s == null || s.TrackingId == 0) continue;

                if (_skeleton == null || _skeleton.TrackingState == SkeletonTrackingState.NotTracked)
                {
                    _skeleton = s;
                    _skeleton.TrackingState = SkeletonTrackingState.Tracked;
                    Console.WriteLine(_skeleton.TrackingId);
                }

                _skeleton = s;

                return;
            }

            _skeleton = null;

        }

        public SkeletonPoint GetJointPos(JointType joint)
        {

            SkeletonPoint point = new SkeletonPoint();

            foreach (Joint j in _skeleton.Joints)
            {
                if (j.JointType == joint)
                {
                    point = j.Position;
                }
            }

            return point;

        }

        public Corgi2 HeadVector
        {
            get
            {

                if (PlayerSkeleton == null)
                    return new Corgi2(0, 0);

                Joint head = new Joint();
                Joint neck = new Joint();

                foreach (Joint j in PlayerSkeleton.Joints)
                {
                    if (j.JointType == JointType.Head)
                        head = j;
                    else if (j.JointType == JointType.ShoulderCenter)
                        neck = j;
                }

                if (Length(head.Position) == 0 && Length(neck.Position) == 0)
                    return new Corgi2(0, 0);


                Corgi2 head_up = new Corgi2(head.Position.X - neck.Position.X, head.Position.Y - neck.Position.Y);
                head_up.Normalize();

                return head_up;

            }
        }

        public float HeadNormAngle
        {
            get
            {
                if (PlayerSkeleton == null)
                    return 0;

                Corgi2 head = HeadNormal;

                Corgi2 plane = new Corgi2(head.X < 0 ? -1 : 1, 0);
                plane.Normalize();

                float theta = (float)head.Dot(plane);
                theta = (float)Math.Acos(theta);


                //Set the correct Sign
                return theta * (head.Y < 0 ? -1 : 1);
            }
        }

        public Corgi2 HeadNormal
        {
            get
            {
                Corgi2 head = HeadVector;
                return new Corgi2(head.Y, -head.X); 
            }
        }


        public float RHRSAngle
        {
            get
            {

                if (PlayerSkeleton == null)
                    return 0;

                Joint shoulder = new Joint();
                Joint hand = new Joint();

                foreach (Joint j in PlayerSkeleton.Joints)
                {
                    if (j.JointType == JointType.ShoulderRight)
                        shoulder = j;
                    else if (j.JointType == JointType.HandRight)
                        hand = j;   
                }

                if (Length(shoulder.Position) == 0 && Length(hand.Position) == 0)
                    return 0;


                Corgi2 arm = new Corgi2(hand.Position.X - shoulder.Position.X, hand.Position.Y - shoulder.Position.Y);
                arm.Normalize();
                Corgi2 plane = new Corgi2(arm.X < 0 ? -1 : 1, 0);
                plane.Normalize();

                float theta = (float)arm.Dot(plane);
                theta = (float)Math.Acos(theta);

                
                //Set the correct Sign
                return theta * (arm.Y < 0 ? -1 : 1);
            }
        }

        public float RPRKAngle
        {
            get
            {

                if (PlayerSkeleton == null)
                    return 0;

                Joint hip = new Joint();
                Joint knee = new Joint();

                foreach (Joint j in PlayerSkeleton.Joints)
                {
                    if (j.JointType == JointType.HipRight)
                        hip = j;
                    else if (j.JointType == JointType.KneeRight)
                        knee = j;
                }

                if (Length(hip.Position) == 0 && Length(knee.Position) == 0)
                    return 0;


                Corgi2 thigh = new Corgi2(knee.Position.X - hip.Position.X, knee.Position.Y - hip.Position.Y);
                thigh.Normalize();
                Corgi2 plane = new Corgi2(thigh.X < 0 ? -1 : 1, 0);
                plane.Normalize();

                float theta = (float)thigh.Dot(plane);
                theta = (float)Math.Acos(theta);

                //Set the correct Sign
                return theta * (thigh.Y < 0 ? -1 : 1);
            }
        }

        public Corgi2 LHLEVector
        {
            get
            {

                if (PlayerSkeleton == null)
                    return new Corgi2(0, 0);

                Joint elbow = new Joint();
                Joint hand = new Joint();

                foreach (Joint j in PlayerSkeleton.Joints)
                {
                    if (j.JointType == JointType.ElbowLeft)
                        elbow = j;
                    else if (j.JointType == JointType.HandLeft)
                        hand = j;
                }

                if (Length(elbow.Position) == 0 && Length(hand.Position) == 0)
                    return new Corgi2(0, 0);


                Corgi2 arm = new Corgi2(hand.Position.X - elbow.Position.X, hand.Position.Y - elbow.Position.Y);
                arm.Normalize();

                return arm;

            }
        }

        public Corgi2 LForearmNorm
        {
            get
            {   
                Corgi2 forearm = LHLEVector;
                return new Corgi2(forearm.Y, -forearm.X); 
            }
        }

        public float UserZ
        {
            get
            {
                return _skeleton.Position.Z;
            }
        }


        public float UserX
        {
            get
            {
                return _skeleton.Position.X;
            }
        }

        #endregion

        #region HELPER
        private float Length(float x, float y, float z)
        {
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }

        private float Length(SkeletonPoint p)
        {
            return (float)Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
        }

        private void print(SkeletonPoint p)
        {
            System.Diagnostics.Debug.WriteLine("pos: [" + p.X + ", " + p.Y + ", " + p.Z + "]");
        }


        #endregion

        #region SPEECH

        private static RecognizerInfo GetKinectRecognizer()
        {
            foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                
                System.Diagnostics.Debug.WriteLine("True".Equals(value, StringComparison.OrdinalIgnoreCase));

                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {

                    return recognizer;
                }
            }

            return null;
        }

        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {

            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            // Number of degrees in a right angle.
            const int DegreesInRightAngle = 90;

            // Number of pixels turtle should move forwards or backwards each time.
            const int DisplacementAmount = 60;

            if (e.Result.Confidence >= ConfidenceThreshold)
            {

                System.Diagnostics.Debug.WriteLine("SPEECH: " + e.Result.Semantics.Value.ToString());

                switch (e.Result.Semantics.Value.ToString())
                {
                    case "SIT": ChangeViewMode("SIT"); break;
                    case "STAND": ChangeViewMode("STAND"); break;
                    default: _col = e.Result.Semantics.Value.ToString(); break;
                }
                
            }
        }

        #endregion

        private Dictionary<int, InteractionHandEventType> _lastLeftHandEvents = new Dictionary<int, InteractionHandEventType>();
        private Dictionary<int, InteractionHandEventType> _lastRightHandEvents = new Dictionary<int, InteractionHandEventType>();

        private void InteractionStreamOnInteractionFrameReady(object sender, InteractionFrameReadyEventArgs args)
        {
            using (var iaf = args.OpenInteractionFrame()) //dispose as soon as possible
            {
                if (iaf == null)
                    return;

                iaf.CopyInteractionDataTo(_userInfos);
            }

            StringBuilder dump = new StringBuilder();

            var hasUser = false;
            foreach (var userInfo in _userInfos)
            {
                var userID = userInfo.SkeletonTrackingId;
                if (userID == 0)
                    continue;

                hasUser = true;
                dump.AppendLine("User ID = " + userID);
                dump.AppendLine("  Hands: ");
                var hands = userInfo.HandPointers;
                if (hands.Count == 0)
                    dump.AppendLine("    No hands");
                else
                {

                   _hand = new Corgi2((float)hands[0].X, (float)hands[0].Y);
                }
            }
        }   

        /// <summary>
        /// Handles the checking or unchecking of the near mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ChangeViewMode(string mode)
        {
            if (Sensor != null)
            {
                try
                {
                    if (mode == "SIT")
                    {
                        Sensor.DepthStream.Range = DepthRange.Near;
                        Sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                    }
                    else
                    {
                        Sensor.DepthStream.Range = DepthRange.Default;
                        Sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                    }
                }
                catch (InvalidOperationException)
                {
                }
            }
        }

        
    }

}


public class Corgi2
{
    public float X = 0, Y = 0;
    public Corgi2(float x, float y)
    {
        X = x;
        Y = y;
    }

    public void Normalize()
    {
        if (Length != 0)
        {
            this.X /= Length;
            this.Y /= Length;
        }
    }

    public float Length
    {
        get
        {
            return (float)Math.Sqrt(X * X + Y * Y);
        }
    }

    public float Dot(Corgi2 p)
    {

        return p.X * X + p.Y * Y;

    }

    public override string ToString()
    {
        return "<" + X + ", " + Y + ">";
    }
}


public class DummyInteractionClient : IInteractionClient
{
    public InteractionInfo GetInteractionInfoAtLocation(
        int skeletonTrackingId,
        InteractionHandType handType,
       double x,
        double y)
    {
        var result = new InteractionInfo();
        result.IsGripTarget = true;
        result.IsPressTarget = true;
        result.PressAttractionPointX = 0.5;
        result.PressAttractionPointY = 0.5;
        result.PressTargetControlId = 1;

        return result;
    }
}


