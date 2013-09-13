﻿using Microsoft.Kinect;
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

        KinectSensor Sensor;
        Skeleton[] _skeletonData;

        Skeleton _skeleton = null;
        SpeechRecognitionEngine SpeechEngine;

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

        public Kinect()
        {
            
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
                Sensor.SkeletonStream.Enable();
                Sensor.SkeletonFrameReady += Sensor_SkeletonFrameReady;
                Sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;

                _skeleton = new Skeleton();

                try
                {
                    Sensor.Start();
                }
                catch (Exception e)
                {
                    Sensor = null;
                }
            }

            RecognizerInfo ri = GetKinectRecognizer();

            if (null != ri)
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

        }


        #region SKELETON
        private void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] _skeletonData = new Skeleton[0];
            using (SkeletonFrame skelFrame = e.OpenSkeletonFrame())
            {
                if (skelFrame != null)
                {
                    _skeletonData = new Skeleton[skelFrame.SkeletonArrayLength];
                    skelFrame.CopySkeletonDataTo(_skeletonData);
                }
            }

            foreach (Skeleton s in _skeletonData)
            {
                if (_skeleton.TrackingState == SkeletonTrackingState.NotTracked && (Length(s.Position) > 0))
                {
                    _skeleton = s;
                    _skeleton.TrackingState = SkeletonTrackingState.Tracked;
                }
                else if (s.TrackingState == SkeletonTrackingState.Tracked && _skeleton != null && s.TrackingId == _skeleton.TrackingId)
                {
                    _skeleton = s;
                }

            }


            System.Diagnostics.Debug.WriteLine(RightHandAngle);

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

        public float RightHandAngle
        {
            get
            {

                if (PlayerSkeleton == null)
                    return 180.0f;

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


                Vector2 arm = new Vector2(hand.Position.X - shoulder.Position.X, hand.Position.Y - shoulder.Position.Y);
                arm.Normalize();
                Vector2 plane = new Vector2(arm.X < 0 ? -1 : 1, 0);
                plane.Normalize();

                float theta = (float)arm.Dot(plane);
                theta = (float)Math.Acos(theta);

                

                return theta * 180.0f / (float)Math.PI;


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

                System.Diagnostics.Debug.WriteLine(e.Result.Semantics.Value.ToString());

                switch (e.Result.Semantics.Value.ToString())
                {
                    case "FORWARD":
                       
                        break;

                    case "BACKWARD":
                       
                        break;
                }
            }
        }

        #endregion

    }
}


public class Vector2
{
    public float X = 0, Y = 0;
    public Vector2(float x, float y)
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

    public float Dot(Vector2 p)
    {

        return p.X * X + p.Y * Y;

    }

}











