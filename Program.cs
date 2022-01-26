using System;
using System.Drawing;
using System.Numerics;
using System.Collections;
using System.Collections.Generic;

namespace DubinsPathsTutorial
{
    class DubinCurevs
    {
        static void Main(string[] args)
        {
            Vector3 goalLeft;
            Vector3 goalRight;
            Vector3 starLeft;
            Vector3 startRight;

            //Objects
            DubinsGeneratePaths dubinsPathGenerator = new DubinsGeneratePaths();
            
            //To generate paths we need the position and rotation (heading) of the cars
            Vector3 startPos = new Vector3(x:6.4362118f, y:0.0f, z:-39.76216f);
            Vector3 goalPos = new Vector3(x:57.883846f, y:0.0f, z:5.1088465f);

            //Heading is in radians
            float startHeading = (180f-123.8365476383409f) * (MathF.PI * 2) / 360;
            float goalHeading = -45 * (MathF.PI * 2) / 360;

            //Get all valid Dubins paths
            List<OneDubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
                startPos, 
                startHeading,
                goalPos,
                goalHeading);

            //Position the left and right circles
            (goalLeft, goalRight, starLeft, startRight) = PositionLeftRightCircle();

            //Position the left and right circle objects for debugging
            (Vector3, Vector3, Vector3, Vector3) PositionLeftRightCircle()
            {
                Vector3 goalLeft;
                Vector3 goalRight;
                Vector3 starLeft;
                Vector3 startRight;

                goalLeft = dubinsPathGenerator.goalLeftCircle;
                goalRight = dubinsPathGenerator.goalRightCircle;

                starLeft = dubinsPathGenerator.startLeftCircle;
                startRight = dubinsPathGenerator.startRightCircle;

                return (goalLeft, goalRight, starLeft, startRight);

            }



            Console.WriteLine("Hello World!");
        }
    }
}
