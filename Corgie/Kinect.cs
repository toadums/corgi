using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Corgie
{
    public class Kinect
    {

        KinectSensor Sensor;
        Skeleton[] _skeletonData;

        Skeleton _skeleton;

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


        }





        private void Sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            _skeletonData = new Skeleton[0];
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
                if (_skeleton == null && (Length(s.Position) > 0))
                {
                    _skeleton = s;
                }
                else if (_skeleton != null && s.TrackingId != _skeleton.TrackingId)
                {
                    _skeleton = s;
                }
            }

        }






        private float Length(float x, float y, float z)
        {
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }

        private float Length(SkeletonPoint p)
        {
            return (float)Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
        }

    }
}














