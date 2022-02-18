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
            Vector3 startPos = new Vector3(x:4.3175286618571f, y:0.0f, z:-41.1824537418076f);
            Vector3 goalPos = new Vector3(x:57.8838464940728f, y:0.0f, z:5.1088464940728f);

            Vector3 sideReturnPos = new Vector3(x:52.775f, y:0.0f, z:0.0f);
            List<(PointF center, PointF cutpoint, char direction)> NewgoalPos = new List<(PointF center, PointF cutpoint, char direction)>();
            NewgoalPos.Add((new PointF(sideReturnPos.X, sideReturnPos.Z), 
                            new PointF(goalPos.X, goalPos.Z),
                             'L'));

            List<Vector3> DetectedShips = new List<Vector3>();
            DetectedShips.Add(new Vector3(x:19.6595995802937f, y:0.0f, z:-17.7597967632823f));
            DetectedShips.Add(new Vector3(x:24.7850164634301f, y:0.0f, z:-24.1341525980847f));
            
            //Heading is in radians(弧度)
            float startHeading = (180f-123.8365476383409f) * (MathF.PI * 2) / 360;
            float goalHeading = -45 * (MathF.PI * 2) / 360;

            
            List<(PointF center, PointF cutpoint, char direction)> NewstartPos = NewStartPos(startPos, startHeading, DetectedShips);


            OneDubinsPath dubin_path_1 = FinalDubinPath(NewstartPos[1], NewgoalPos[0], sideReturnPos, DetectedShips, startHeading, goalHeading);

            Console.WriteLine("Hello World!");
        }

        public static List<(PointF , PointF , char)> NewStartPos(Vector3 startPos, float startHeading, List<Vector3> DetectedShips)
        {
            float return_radius = 7.225f;
            float threaten_radius = 28.0f;

            float ToOrgAngleAxis = -startHeading + (MathF.PI/2);
            Vector2 HeadingVec = MathFunction.GetVector(ToOrgAngleAxis, 1);

            Vector2 LeftVec = new Vector2(x: -HeadingVec.Y, y:HeadingVec.X);
            Vector2 RightVec = new Vector2(x:HeadingVec.Y, y:-HeadingVec.X); 

            PointF LeftReturnCenter = new PointF(startPos.X + return_radius * LeftVec.X,
                                                startPos.Z + return_radius * LeftVec.Y);

            PointF RightReturnCenter = new PointF(startPos.X + return_radius * RightVec.X,
                                                startPos.Z + return_radius * RightVec.Y);
            
            // Produce new left return circle
            PointF NewLeftReturnCircle = new PointF(LeftReturnCenter.X, LeftReturnCenter.Y);
            for (int i = 0; i<DetectedShips.Count; i++)
            {
                PointF detectedship = new PointF(x:DetectedShips[i].X, y:DetectedShips[i].Z);
                double ReturnToShip = MathFunction.Distance(NewLeftReturnCircle, detectedship);

                if(ReturnToShip < threaten_radius + return_radius)
                {
                    PointF lineEnd = new PointF(NewLeftReturnCircle.X - 100 * HeadingVec.X,
                                                NewLeftReturnCircle.Y - 100 * HeadingVec.Y);

                    NewLeftReturnCircle = MathFunction.ClosestIntersection(detectedship.X, detectedship.Y, threaten_radius+return_radius, NewLeftReturnCircle, lineEnd);

                }
            }
            // Produce new left return circle
            PointF NewLeftReturnCutPoint = new PointF(NewLeftReturnCircle.X + return_radius * RightVec.X,
                                                    NewLeftReturnCircle.Y + return_radius * RightVec.Y);

            // Produce new right return circle
            PointF NewRightReturnCircle = new PointF(RightReturnCenter.X, RightReturnCenter.Y);
            for (int i = 0; i<DetectedShips.Count; i++)
            {
                PointF detectedship = new PointF(x:DetectedShips[i].X, y:DetectedShips[i].Z);
                double ReturnToShip = MathFunction.Distance(NewRightReturnCircle, detectedship);

                if(ReturnToShip < threaten_radius + return_radius)
                {
                    PointF lineEnd = new PointF(NewRightReturnCircle.X - 100 * HeadingVec.X,
                                                NewRightReturnCircle.Y - 100 * HeadingVec.Y);

                    NewRightReturnCircle = MathFunction.ClosestIntersection(detectedship.X, detectedship.Y, threaten_radius+return_radius, NewRightReturnCircle, lineEnd);

                }
            }
            // Produce new left return circle
            PointF NewRightReturnCutPoint = new PointF(NewRightReturnCircle.X + return_radius * LeftVec.X,
                                                    NewRightReturnCircle.Y + return_radius * LeftVec.Y);


            List<(PointF center, PointF cutpoint, char direction)> NewstartPos = new List<(PointF center, PointF cutpoint, char direction)>();
            NewstartPos.Add((NewLeftReturnCircle, NewLeftReturnCutPoint, 'L'));
            NewstartPos.Add((NewRightReturnCircle, NewRightReturnCutPoint, 'R'));

            return NewstartPos;

        }
        public static OneDubinsPath FinalDubinPath((PointF, PointF, char) NewstartPos, (PointF, PointF, char) NewgoalPos, Vector3 sideReturnPos, 
                                                List<Vector3> DetectedShips, float startHeading, float goalHeading)
        {
            Vector3 goalLeft;
            Vector3 goalRight;
            Vector3 startLeft;
            Vector3 startRight;

            //Objects
            DubinsGeneratePaths dubinsPathGenerator = new DubinsGeneratePaths();

            // Item2 is cutpoint
            Vector3 startPos = new Vector3(x:NewstartPos.Item2.X, y:0.0f, z:NewstartPos.Item2.Y);
            Vector3 goalPos = new Vector3(x:NewgoalPos.Item2.X, y:0.0f, z:NewgoalPos.Item2.Y);
            //Get all valid Dubins paths
            List<OneDubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
                startPos, 
                startHeading,
                goalPos,
                goalHeading);

            
            // Position the left and right circles
            goalLeft = dubinsPathGenerator.goalLeftCircle;
            goalRight = dubinsPathGenerator.goalRightCircle;

            startLeft = dubinsPathGenerator.startLeftCircle;
            startRight = dubinsPathGenerator.startRightCircle;

            // Choose the target circle
            // Remove the dubin path witch is wrong direction of start pos
            for(int i = pathDataList.Count-1; i >= 0; i--)
            {
                if (pathDataList[i].pathType.ToString()[0] != NewstartPos.Item3 || 
                    pathDataList[i].pathType.ToString()[2] != NewgoalPos.Item3)
                {
                    pathDataList.RemoveAt(i);
                }

            }

            // Item1 is center
            Vector3 startCenter = new Vector3(x:NewstartPos.Item1.X, y:0.0f, z:NewstartPos.Item1.Y);
            Vector3 goalCenter = new Vector3(x:NewgoalPos.Item1.X, y:0.0f, z:NewgoalPos.Item1.Y);
            MathFunction.Circle firstavoidancecircle = MathFunction.AllReturnCircle(startCenter, goalCenter, pathDataList[0], DetectedShips);


            //Choose the return circle
            // for(int i = pathDataList.Count -1; i >=0; i--)
            // {
            //     //複製所有已觀測到的船艦
            //     List<Vector3> ExecuteShips = new List<Vector3>(DetectedShips);

            //     //x:該船艦投影到切線的投影點，到迴轉圓切點(tangent1)的距離
            //     //y:該船艦到飛彈當前位置的距離
            //     List<Vector2> ExecuteShips_Dist = new List<Vector2>();

            //     double TangentDist =  MathFunction.Distance(new PointF(pathDataList[i].tangent1.X, pathDataList[i].tangent1.Z),
            //                                                 new PointF(pathDataList[i].tangent2.X, pathDataList[i].tangent2.Z));

            //     // 從已觀測的船艦資訊中，將不在切線線段內的護衛艦剔除
            //     for(int j = ExecuteShips.Count-1; j >= 0; j--)
            //     {
            //         PointF ProjectivePoint = MathFunction.LinePointProjection(new PointF(pathDataList[i].tangent1.X, pathDataList[i].tangent1.Z),
            //                                                                 new PointF(pathDataList[i].tangent2.X, pathDataList[i].tangent2.Z),
            //                                                                 new PointF(ExecuteShips[j].X, ExecuteShips[j].Z));

            //         double ProjectivePoint_Tangent1 = MathFunction.Distance(new PointF(pathDataList[i].tangent1.X, pathDataList[i].tangent1.Z), ProjectivePoint);

            //         double ProjectivePoint_Tangent2 = MathFunction.Distance(new PointF(pathDataList[i].tangent2.X, pathDataList[i].tangent2.Z), ProjectivePoint);

            //         double Missile_Ship = MathFunction.Distance(new PointF(ExecuteShips[j].X, ExecuteShips[j].Z),
            //                                                     new PointF(startPos.X, startPos.Z));

            //         if (ProjectivePoint_Tangent1 + ProjectivePoint_Tangent2 > TangentDist)
            //         {
            //             ExecuteShips.RemoveAt(j);
            //         }
            //         else
            //         {
            //             ExecuteShips_Dist.Add(new Vector2(x:(float)ProjectivePoint_Tangent1, y:(float)Missile_Ship));
            //         }

            //     }
            //     // 因為刪除過程是從後面往前刪，而距離存入過程是依序Add進入ExecuteShips_Dist，故保留下的船艦所對應的距離要倒序
            //     ExecuteShips_Dist.Reverse();

            //     float cloest_projective_dist = float.MaxValue;
            //     int closet_ship_indx = 0;
            //     for (int j=0; j<ExecuteShips.Count; j++)
            //     {
            //         if (ExecuteShips_Dist[j].X < cloest_projective_dist && ExecuteShips_Dist[j].Y <= 28.0f)
            //         {
            //             cloest_projective_dist = ExecuteShips_Dist[j].X;
            //             closet_ship_indx = j;
            //         }

            //     }

            //     int ship_side = MathFunction.SideOfVector(new PointF(pathDataList[i].tangent1.X, pathDataList[i].tangent1.Z),
            //                                             new PointF(pathDataList[i].tangent2.X, pathDataList[i].tangent2.Z),
            //                                             new PointF(shipPos.X, shipPos.Z));

            //     char return_side;
                
            //     // 若護衛艦在直線向量左邊，則採用右迴轉圓
            //     if (ship_side >= 0) return_side = 'R';

            //     //若護衛艦在直線向量右邊，則採用左迴轉圓
            //     else return_side = 'L';

            //     if (pathDataList[i].pathType.ToString()[0] != return_side)
            //     {
            //         pathDataList.RemoveAt(i);
            //     }

            // }

            return pathDataList[0];
            
        }

    }
}
