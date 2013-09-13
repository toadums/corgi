using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Corgie
{
    class CSkeleton: Skeleton
    {


        public override string ToString()
        {

            return "Pos: [" + Position.X + ", " + Position.Y + ", " + Position.Z + "]";
        }

    }
}
