using System;
using System.Diagnostics;
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

            Vector3 leftsideReturnPos = new Vector3(x:52.775f, y:0.0f, z:0.0f);
            Vector3 rightsideReturnPos = new Vector3(x:62.9926929881455f, y:0.0f, z:10.2176929881455f);
            List<(PointF center, PointF cutpoint, char direction)> NewgoalPos = new List<(PointF center, PointF cutpoint, char direction)>();
            NewgoalPos.Add((new PointF(leftsideReturnPos.X, leftsideReturnPos.Z), 
                            new PointF(goalPos.X, goalPos.Z),
                             'L'));
            NewgoalPos.Add((new PointF(rightsideReturnPos.X, rightsideReturnPos.Z), 
                            new PointF(goalPos.X, goalPos.Z),
                             'R'));

            List<Vector3> DetectedShips = new List<Vector3>();
            DetectedShips.Add(new Vector3(x:24.7106555175762f, y:0.0f, z:-21.9959903614306f));
            DetectedShips.Add(new Vector3(x:82.7536255633538f, y:0.0f, z:-22.1092239774434f));
            // DetectedShips.Add(new Vector3(x:72.3815707560877f, y:0.0f, z:-30.8055751609168f));
            
            //Heading is in radians(弧度)
            float startHeading = (180f-123.8365476383409f) * (MathF.PI * 2) / 360;
            float goalHeading = -45 * (MathF.PI * 2) / 360;

            Stopwatch sw = new Stopwatch();
            sw.Start();

            // 回傳新的左右迴轉圓，資料結構為(圓心、切點、迴轉方向)，[0]為左迴轉、[1]為右回轉
            List<(PointF center, PointF cutpoint, char direction)> NewstartPos = NewStartPos(startPos, startHeading, DetectedShips);

            List<List<Tuple<MathFunction.Circle, char>>> right_left = FinalDubinPath(NewstartPos[1], NewgoalPos[0], leftsideReturnPos, DetectedShips, startHeading, goalHeading);
            List<List<Tuple<MathFunction.Circle, char>>> left_left = FinalDubinPath(NewstartPos[0], NewgoalPos[0], leftsideReturnPos, DetectedShips, startHeading, goalHeading);
            List<List<Tuple<MathFunction.Circle, char>>> right_right = FinalDubinPath(NewstartPos[1], NewgoalPos[1], rightsideReturnPos, DetectedShips, startHeading, goalHeading);
            List<List<Tuple<MathFunction.Circle, char>>> left_right = FinalDubinPath(NewstartPos[0], NewgoalPos[1], rightsideReturnPos, DetectedShips, startHeading, goalHeading);
            sw.Stop();
            TimeSpan ts2 = sw.Elapsed;  
            Console.WriteLine("Stopwatch總共花費{0}ms.", ts2.TotalMilliseconds);  
        }

        public static List<(PointF , PointF , char)> NewStartPos(Vector3 startPos, float startHeading, List<Vector3> DetectedShips)
        {
            float return_radius = 7.225f;
            float threaten_radius = 28.0f;

            // 將Unity座標軸：北(0)、西(-90)、東(90)、南(+-180)，轉換為標準座標軸：東(0)、北(90)、西(180)、南(270)
            float ToOrgAngleAxis = -startHeading + (MathF.PI/2);
            // 當前航行角度轉換成單位向量
            Vector2 HeadingVec = MathFunction.GetVector(ToOrgAngleAxis, 1);

            // 航行向量的左邊法向量
            Vector2 LeftVec = new Vector2(x: -HeadingVec.Y, y:HeadingVec.X);
            // 航行向量的右邊法向量
            Vector2 RightVec = new Vector2(x:HeadingVec.Y, y:-HeadingVec.X); 
            
            // 飛彈當強位置的左迴轉圓圓心
            PointF LeftReturnCenter = new PointF(startPos.X + return_radius * LeftVec.X,
                                                startPos.Z + return_radius * LeftVec.Y);
            // 飛彈當強位置的右迴轉圓圓心
            PointF RightReturnCenter = new PointF(startPos.X + return_radius * RightVec.X,
                                                startPos.Z + return_radius * RightVec.Y);
            
            // Produce new left return circle, it is same as original one at the begining.
            PointF NewLeftReturnCircle = new PointF(LeftReturnCenter.X, LeftReturnCenter.Y);
            for (int i = 0; i<DetectedShips.Count; i++)
            {   

                PointF detectedship = new PointF(x:DetectedShips[i].X, y:DetectedShips[i].Z);
                // 迴轉圓圓心至當前護衛艦距離
                double ReturnToShip = MathFunction.Distance(NewLeftReturnCircle, detectedship);
                // 若迴轉圓與護衛艦威脅圓重疊
                if(ReturnToShip < threaten_radius + return_radius)
                {
                    //線段起點為當前迴轉圓位置
                    //線段終點為當前迴轉位置-100倍的相反航行方向
                    PointF lineEnd = new PointF(NewLeftReturnCircle.X - 100 * HeadingVec.X,
                                                NewLeftReturnCircle.Y - 100 * HeadingVec.Y);

                    // 線段與"以護衛艦為圓心，半徑為28+7.225的圓"，所產生之交點，這邊使用距離護衛艦圓心較近的交點最為新回轉圓
                    NewLeftReturnCircle = MathFunction.ClosestIntersection(detectedship.X, detectedship.Y, threaten_radius+return_radius, NewLeftReturnCircle, lineEnd);

                }
            }
            // 新左迴轉圓與原航行方向的切點
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
            // 新右迴轉圓與原航行方向的切點
            PointF NewRightReturnCutPoint = new PointF(NewRightReturnCircle.X + return_radius * LeftVec.X,
                                                    NewRightReturnCircle.Y + return_radius * LeftVec.Y);


            List<(PointF center, PointF cutpoint, char direction)> NewstartPos = new List<(PointF center, PointF cutpoint, char direction)>();
            NewstartPos.Add((NewLeftReturnCircle, NewLeftReturnCutPoint, 'L'));
            NewstartPos.Add((NewRightReturnCircle, NewRightReturnCutPoint, 'R'));

            return NewstartPos;

        }

        /// <summary>  
        /// 給定迴轉圓圓心、目標圓圓心、迴轉圓切點、目標圓切點，輸出RSR的Dubin曲線
        /// </summary>
        public static OneDubinsPath GetRSR_OneDubinsPath(Vector3 startcenter, Vector3 goalcenter, Vector3 startPos, Vector3 goalPos)
        {
            //Find both tangent positons
            Vector3 startTangent = Vector3.Zero;
            Vector3 goalTangent = Vector3.Zero;
            DubinsMath.LSLorRSR(startcenter, goalcenter, false, out startTangent, out goalTangent);
            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startcenter, startPos, startTangent, false);
            float length2 = (startTangent - goalTangent).Length();
            float length3 = DubinsMath.GetArcLength(goalcenter, goalTangent, goalPos, false);
            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSR);
            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;
            //RSR
            pathData.SetIfTurningRight(true, false, true);

            return pathData;

        }
        
        /// <summary>  
        /// 給定迴轉圓圓心、目標圓圓心、迴轉圓切點、目標圓切點，輸出LSL的Dubin曲線
        /// </summary>
        public static OneDubinsPath GetLSL_OneDubinsPath(Vector3 startcenter, Vector3 goalcenter, Vector3 startPos, Vector3 goalPos)
        {
            //Find both tangent positons
            Vector3 startTangent = Vector3.Zero;
            Vector3 goalTangent = Vector3.Zero;
            DubinsMath.LSLorRSR(startcenter, goalcenter, true, out startTangent, out goalTangent);
            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startcenter, startPos, startTangent, true);
            float length2 = (startTangent - goalTangent).Length();
            float length3 = DubinsMath.GetArcLength(goalcenter, goalTangent, goalPos, true);
            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSL);
            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;
            //LSL
            pathData.SetIfTurningRight(false, false, false);

            return pathData;

        }
        
        /// <summary>  
        /// 給定迴轉圓圓心、目標圓圓心、迴轉圓切點、目標圓切點，輸出RSL的Dubin曲線
        /// </summary>
        public static OneDubinsPath GetRSL_OneDubinsPath(Vector3 startcenter, Vector3 goalcenter, Vector3 startPos, Vector3 goalPos)
        {
            //Find both tangent positons
            Vector3 startTangent = Vector3.Zero;
            Vector3 goalTangent = Vector3.Zero;
            DubinsMath.RSLorLSR(startcenter, goalcenter, false, out startTangent, out goalTangent);
            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startcenter, startPos, startTangent, false);
            float length2 = (startTangent - goalTangent).Length();
            float length3 = DubinsMath.GetArcLength(goalcenter, goalTangent, goalPos, true);
            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.RSL);
            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;
            //RSL
            pathData.SetIfTurningRight(true, false, false);

            return pathData;
        }
        
        /// <summary>  
        /// 給定迴轉圓圓心、目標圓圓心、迴轉圓切點、目標圓切點，輸出LSR的Dubin曲線
        /// </summary>
        public static OneDubinsPath GetLSR_OneDubinsPath(Vector3 startcenter, Vector3 goalcenter, Vector3 startPos, Vector3 goalPos)
        {
            //Find both tangent positons
            Vector3 startTangent = Vector3.Zero;
            Vector3 goalTangent = Vector3.Zero;
            DubinsMath.RSLorLSR(startcenter, goalcenter, true, out startTangent, out goalTangent);
            //Calculate lengths
            float length1 = DubinsMath.GetArcLength(startcenter, startPos, startTangent, true);
            float length2 = (startTangent - goalTangent).Length();
            float length3 = DubinsMath.GetArcLength(goalcenter, goalTangent, goalPos, false);
            //Save the data
            OneDubinsPath pathData = new OneDubinsPath(length1, length2, length3, startTangent, goalTangent, PathType.LSR);
            //We also need this data to simplify when generating the final path
            pathData.segment2Turning = false;
            //LSR
            pathData.SetIfTurningRight(false, false, true);

            return pathData;
        }

        public static List<List<Tuple<MathFunction.Circle, char>>> FinalDubinPath((PointF, PointF, char) NewstartPos, (PointF, PointF, char) NewgoalPos, Vector3 sideReturnPos, 
                                                List<Vector3> DetectedShips, float startHeading, float goalHeading)
        {
            
            //Objects
            DubinsGeneratePaths dubinsPathGenerator = new DubinsGeneratePaths();

            // Item1 is center
            Vector3 startcenter = new Vector3(x:NewstartPos.Item1.X, y:0.0f, z:NewstartPos.Item1.Y);
            Vector3 goalcenter = new Vector3(x:NewgoalPos.Item1.X, y:0.0f, z:NewgoalPos.Item1.Y);
            
            // Item2 is cutpoint
            Vector3 startPos = new Vector3(x:NewstartPos.Item2.X, y:0.0f, z:NewstartPos.Item2.Y);
            Vector3 goalPos = new Vector3(x:NewgoalPos.Item2.X, y:0.0f, z:NewgoalPos.Item2.Y);
            
            //Get all valid Dubins paths
            // List<OneDubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
            //     startPos, 
            //     startHeading,
            //     goalPos,
            //     goalHeading);

            
            // Position the left and right circles
            // Vector3 goalLeft = dubinsPathGenerator.goalLeftCircle;
            // Vector3 goalRight = dubinsPathGenerator.goalRightCircle;
            // Vector3 startLeft = dubinsPathGenerator.startLeftCircle;
            // Vector3 startRight = dubinsPathGenerator.startRightCircle;

            // Choose the target circle
            // Remove the dubin path witch is wrong direction of start pos
            // for(int i = pathDataList.Count-1; i >= 0; i--)
            // {
            //     if (pathDataList[i].pathType.ToString()[0] != NewstartPos.Item3 || 
            //         pathDataList[i].pathType.ToString()[2] != NewgoalPos.Item3)
            //     {
            //         pathDataList.RemoveAt(i);
            //     }

            // }

            // 根據迴轉圓與目標圓的方向，製造出對應的dubin曲線路徑
            List<OneDubinsPath> pathDataList = new List<OneDubinsPath>();
            OneDubinsPath pathData;
            if (NewstartPos.Item3 == NewgoalPos.Item3)
            {
                //RSR and LSL is only working if the circles don't have the same position
                if (startcenter.X != goalcenter.X && startcenter.Z != goalcenter.Z)
                {
                    // RSR
                    if (NewstartPos.Item3 == 'R')
                    {
                        pathData = GetRSR_OneDubinsPath(startcenter, goalcenter, startPos, goalPos);
                    }
                    // LSL
                    else
                    {
                        pathData = GetLSL_OneDubinsPath(startcenter, goalcenter, startPos, goalPos);
                    }
                    //  Add the path to the collection of all paths
                    pathDataList.Add(pathData);
                }
            }
            else
            {
                //RSL and LSR is only working of the circles don't intersect
                float comparisonSqr = DubinsMath.turningRadius * 2f * DubinsMath.turningRadius * 2f;
                if ((startcenter - goalcenter).LengthSquared() > comparisonSqr)
                {
                    // RSL
                    if (NewstartPos.Item3 == 'R')
                    {
                        pathData = GetRSL_OneDubinsPath(startcenter, goalcenter, startPos, goalPos);
                    }
                    // LSR
                    else
                    {
                        pathData = GetLSR_OneDubinsPath(startcenter, goalcenter, startPos, goalPos);
                    }
                    //  Add the path to the collection of all paths
                    pathDataList.Add(pathData);
                }
            }

            List<List<Tuple<MathFunction.Circle, char>>> list_all_return_circles = MathFunction.AllReturnCircle(startcenter, goalcenter, pathDataList[0], DetectedShips);


            return list_all_return_circles;
            
        }

    }
}
