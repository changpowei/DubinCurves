using System;
using System.Drawing;
using System.Numerics;
using System.Collections;
using System.Collections.Generic;

namespace DubinsPathsTutorial
{
    public class DubinCurves
    {
        public static void Main(string[] args)
        {
            
            //To generate paths we need the position and rotation (heading) of the cars
            Vector3 startPos = new Vector3(x:4.1596642523727f, y:0.0f, z:-41.288280641173f);
            Vector3 goalPos = new Vector3(x:57.883846f, y:0.0f, z:5.1088465f);

            //Heading is in radians
            float startHeading = (180f-123.8365476383409f) * (MathF.PI * 2) / 360;
            float goalHeading = -45 * (MathF.PI * 2) / 360;

            Vector3 shipPos = new Vector3(x:30.619333844083f, y:0.0f, z:-32.1293568453594f);
            Vector3 sideReturnPos = new Vector3(x:52.775f, y:0.0f, z:0.0f);
            
            FinalDubinPath(startPos, goalPos, shipPos, sideReturnPos, startHeading, goalHeading);

            Console.WriteLine("Hello World!");
        }

        public static void FinalDubinPath(Vector3 startPos, Vector3 goalPos, Vector3 shipPos,
                                            Vector3 sideReturnPos, float startHeading, float goalHeading)
        {
            Vector3 goalLeft;
            Vector3 goalRight;
            Vector3 starLeft;
            Vector3 startRight;

            //Objects
            DubinsGeneratePaths dubinsPathGenerator = new DubinsGeneratePaths();

            //Get all valid Dubins paths
            List<OneDubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
                startPos, 
                startHeading,
                goalPos,
                goalHeading);

            //Position the left and right circles
            goalLeft = dubinsPathGenerator.goalLeftCircle;
            goalRight = dubinsPathGenerator.goalRightCircle;

            starLeft = dubinsPathGenerator.startLeftCircle;
            startRight = dubinsPathGenerator.startRightCircle;

            //Choose the target circle
            for(int i = pathDataList.Count-1; i >= 0; i--)
            {
                float distance = (float)Math.Sqrt(Math.Pow(pathDataList[i].tangent2.X - sideReturnPos.X, 2) + 
                                                    Math.Pow(pathDataList[i].tangent2.Z - sideReturnPos.Z, 2));

                if (Math.Abs(distance - 7.225) > 0.00001)
                {
                    pathDataList.RemoveAt(i);
                }

            }

            
        }

    }
}
