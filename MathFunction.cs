using System;
using System.Drawing;
using System.Numerics;
using System.Collections;
using System.Collections.Generic;

namespace DubinsPathsTutorial
{
    public static class MathFunction
    {        
        public class Line{

            public PointF PointA;

            public PointF PointB;

            public Line(PointF PointA, PointF PointB){
                this.PointA = PointA;
                this.PointB = PointB;
            }

        }

        public class Circle{
            public PointF center;
            public float radius;

            public Circle(PointF center, float radius){
                this.center = center;
                this.radius = radius;
            }

        }

        
        private static double Distance(PointF p1, PointF p2)
        {
            return Math.Sqrt(Math.Pow(p2.X - p1.X, 2) + Math.Pow(p2.Y - p1.Y, 2));
        }


        //点到线段距离  
        private static double pointToLine(Line l1, PointF p1)
        {    
            double space = 0;    
            double a, b, c;    
            a = Distance(l1.PointA, l1.PointB);// 线段的长度    
            b = Distance(l1.PointA, p1);// (x1,y1)到点的距离    
            c = Distance(l1.PointB, p1);// (x2,y2)到点的距离    
            if (c <= 0.000001 || b <= 0.000001) {    
                space = 0;    
                return space;    
            }    
            if (a <= 0.000001) {    
                space = b;    
                return space;    
            }    
            if (c * c >= a * a + b * b) {    
                space = b;    
                return space;    
            }    
            if (b * b >= a * a + c * c) {    
                space = c;    
                return space;    
            }    
                double p = (a + b + c) / 2;// 半周长    
                double s = Math.Sqrt(p * (p - a) * (p - b) * (p - c));// 海伦公式求面积    
            space = 2 * s / a;// 返回点到线的距离（利用三角形面积公式求高）    
            return space;    
        }

        /// <summary>
        /// 判斷點在線的左邊還是右邊，兩點p1(x1,y1),p2(x2,y2),判斷點p(x,y)在線的左邊還是右邊
        /// </summary>
        /// <returns>-1:p在線的左邊; 1:p在線的右邊; 0:p點為(Nan, Nan)</returns>
        private static int SideOfLine(PointF p, PointF p1, PointF p2)
        {
            if (double.IsNaN(p.X) || double.IsNaN(p.Y)){
                return 0;
            }
            else{
                double tmpx = (p1.X - p2.X) / (p1.Y - p2.Y) * (p.Y - p2.Y) + p2.X;

                if (tmpx > p.X)//當tmpx>p.x的時候，說明點在線的左邊，小於在右邊，等於則在線上。
                    return -1;
                return 1;

            }
        }

        /// <summary>
        /// 給定三個座標點A,B,C，判斷AC向量在AB的左邊或右邊
        /// </summary>
        /// <returns>右邊:-1;左邊:1</returns>
        private static int SideOfVector(PointF A, PointF B, PointF C)
        {
            Vector2 ab = new Vector2(B.X - A.X, B.Y - A.Y);
            Vector2 ac = new Vector2(C.X - A.X, C.Y - A.Y);

            float cross_product = ab.X * ac.Y - ac.X * ab.Y;

            // cross_product >= 0, AC向量AB的左邊, return 1
            if (cross_product >= 0) return 1;
            // cross_product < 0, AC向量AB的右邊, return -1
            else return -1;
        }

        /// <summary>
        /// 計算直線與圓最近的交點
        /// 圆, 圆心(cx, cy), 半径radius.
        /// lineStart 線段起點
        /// lineEnd 線段終點
        /// </summary>
        /// <returns>返回直線與圓最近的交點 (PointF.X, PointF.Y)</returns>
        public static PointF ClosestIntersection(float cx, float cy, float radius, PointF lineStart, PointF lineEnd)
        {
            PointF intersection1;
            PointF intersection2;
            int intersections = FindLineCircleIntersections(cx, cy, radius, lineStart, lineEnd, out intersection1, out intersection2);

            // 交於一點(相切)
            if (intersections == 1)
                return intersection1;
            // 交於兩點
            else if (intersections == 2)
            {
                double dist1 = Distance(intersection1, lineStart);
                double dist2 = Distance(intersection2, lineStart);

                if (dist1 < dist2)
                    return intersection1;
                else
                    return intersection2;
            }
            // 沒有交點
            else{
                return PointF.Empty; // no intersections at all
            }
        }

        // Find the points of intersection.
        private static int FindLineCircleIntersections(float cx, float cy, float radius, PointF point1, PointF point2, 
                                                out PointF intersection1, out PointF intersection2)
        {
            float dx, dy, A, B, C, det, t;

            dx = point2.X - point1.X;
            dy = point2.Y - point1.Y;

            A = dx * dx + dy * dy;
            B = 2 * (dx * (point1.X - cx) + dy * (point1.Y - cy));
            C = (point1.X - cx) * (point1.X - cx) + (point1.Y - cy) * (point1.Y - cy) - radius * radius;

            det = B * B - 4 * A * C;
            if ((A <= 0.0000001) || (det < 0))
            {
                // No real solutions.
                intersection1 = new PointF(float.NaN, float.NaN);
                intersection2 = new PointF(float.NaN, float.NaN);
                return 0;
            }
            else if (det == 0)
            {
                // One solution.
                t = -B / (2 * A);
                intersection1 = new PointF(point1.X + t * dx, point1.Y + t * dy);
                intersection2 = new PointF(float.NaN, float.NaN);
                return 1;
            }
            else
            {
                // Two solutions.
                t = (float)((-B + Math.Sqrt(det)) / (2 * A));
                intersection1 = new PointF(point1.X + t * dx, point1.Y + t * dy);
                t = (float)((-B - Math.Sqrt(det)) / (2 * A));
                intersection2 = new PointF(point1.X + t * dx, point1.Y + t * dy);
                return 2;
            }
        }

        /// <summary>
        /// 计算两个相离的圆的内公切线。（相交没有内公切线，只有外公切线）
        /// 圆C1, 圆心(a, b), 半径r1.
        /// 圆C2, 圆心(c, d), 半径r2.
        /// </summary>
        /// <returns>返回两条内公切线段，线段的两个端点是圆上的切点, 每條線段的順序為C1的切點，再來才是C2的切點。</returns>
        public static (Line l1, Line l2) InnerTagentLines(Circle circle1, Circle circle2)
        {
            double a = circle1.center.X;
            double b = circle1.center.Y;
            double r1 = circle1.radius;

            double c = circle2.center.X;
            double d = circle2.center.Y;
            double r2 = circle2.radius;

            var r3 = r1 + r2;
            var sigma_1 = Math.Sqrt(a * a - 2 * a * c + b * b - 2 * b * d + c * c + d * d - r3 * r3);
            var sigma_2 = (a - c) * (a * a - 2 * a * c + b * b - 2 * b * d + c * c + d * d);
            var sigma_3 = (-a * a + c * a - b * b + d * b + r3 * r3) / (a - c);
            var sigma_4 = 2 * b * b * d;

            // 计算C3切点(x3_1, y3_1), (x3_2, y3_2)
            var x3_1 = -sigma_3 - (b - d) * (a * a * b + b * c * c + b * d * d - sigma_4 - b * r3 * r3 + d * r3 * r3 + b * b * b + a * r3 * sigma_1 - c * r3 * sigma_1 - 2 * a * b * c) / sigma_2;
            var x3_2 = -sigma_3 - (b - d) * (a * a * b + b * c * c + b * d * d - sigma_4 - b * r3 * r3 + d * r3 * r3 + b * b * b - a * r3 * sigma_1 + c * r3 * sigma_1 - 2 * a * b * c) / sigma_2;

            sigma_1 = Math.Sqrt(a * a - 2 * a * c + b * b - 2 * b * d + c * c + d * d - r3 * r3);
            sigma_2 = a * a - 2 * a * c + b * b - 2 * b * d + c * c + d * d;
            sigma_3 = 2 * b * b * d;

            var y3_1 = (a * a * b + b * c * c + b * d * d - sigma_3 - b * r3 * r3 + d * r3 * r3 + b * b * b + a * r3 * sigma_1 - c * r3 * sigma_1 - 2 * a * b * c) / sigma_2;
            var y3_2 = (a * a * b + b * c * c + b * d * d - sigma_3 - b * r3 * r3 + d * r3 * r3 + b * b * b - a * r3 * sigma_1 + c * r3 * sigma_1 - 2 * a * b * c) / sigma_2;
            
            // 计算C2切点(x2_1, y2_1, x2_2, y2_2)
            var λ = r2 / r3;
            var x2_1 = λ * a + (1 - λ) * x3_1;
            var y2_1 = λ * b + (1 - λ) * y3_1;
            var x2_2 = λ * a + (1 - λ) * x3_2;
            var y2_2 = λ * b + (1 - λ) * y3_2;

            // 计算C1切点(x1_1, y1_1), （x2_1, y2_1)
            var x1_1 = x2_1 - x3_1 + c;
            var y1_1 = y2_1 - y3_1 + d;
            var x1_2 = x2_2 - x3_2 + c;
            var y1_2 = y2_2 - y3_2 + d;


            Line l1 = new Line(new PointF((float)x2_1, (float)y2_1), new PointF((float)x1_1, (float)y1_1));
            Line l2 = new Line(new PointF((float)x2_2, (float)y2_2), new PointF((float)x1_2, (float)y1_2));
            return (l1, l2);
        }
        
        /// <summary>
        /// 計算兩圓circle1與circle2的外公切線，並返回兩外公切線，每條線由兩切點組成
        /// </summary>
        /// <param name="circle1">圓1</param>
        /// <param name="circle2">圓2</param>
        /// <returns>(Line l1, Line l2)</returns>
        public static (Line l1, Line l2) OuterTagentLines(Circle circle1, Circle circle2)
        {
            //兩外公切線
            Line l1, l2;

            PointF[] CutPoints = new PointF[2];
            if (circle1.radius != circle2.radius){
                (l1, l2) = CalculateForDifferentRadius(circle1, circle2);
                return (l1, l2);
            }else{
                Vector2 c1_c2_vector = Vector2.Normalize(new Vector2(circle2.center.X-circle1.center.X, circle2.center.Y-circle1.center.Y));
                Vector2 left_normal_vector = new Vector2(-c1_c2_vector.Y, c1_c2_vector.X);
                Vector2 right_normal_vector = new Vector2(c1_c2_vector.Y, -c1_c2_vector.X);

                PointF left_point_1 = new PointF(circle1.center.X + circle1.radius * left_normal_vector.X,
                                                circle1.center.Y + circle1.radius * left_normal_vector.Y);

                PointF left_point_2 = new PointF(circle2.center.X + circle2.radius * left_normal_vector.X,
                                                circle2.center.Y + circle2.radius * left_normal_vector.Y);

                PointF right_point_1 = new PointF(circle1.center.X + circle1.radius * right_normal_vector.X,
                                                circle1.center.Y + circle1.radius * right_normal_vector.Y);

                PointF right_point_2 = new PointF(circle2.center.X + circle2.radius * right_normal_vector.X,
                                                circle2.center.Y + circle2.radius * right_normal_vector.Y);               

                
                l1 = new Line(new PointF((float)left_point_1.X, (float)left_point_1.Y), new PointF((float)left_point_2.X, (float)left_point_2.Y));
                l2 = new Line(new PointF((float)right_point_1.X, (float)right_point_1.Y), new PointF((float)right_point_2.X, (float)right_point_2.Y));

                return (l1, l2);
            }
        }

        /// <summary>
        /// 在半径不相等的情况下,计算 circle1 和 circle2 两圆的外公切线
        /// </summary>
        /// <param name="circle1">圓1</param>
        /// <param name="circle2">圓2</param>
        public static (Line l1, Line l2) CalculateForDifferentRadius(Circle circle1, Circle circle2)
        {
            //令circle1，circle2的外公切线交点P
            //circle1的圆心为O1,circle2的圆心为O2         
            //切线于circle1的两个焦点分别记作A1,A2
            //切线于circle2的两个焦点分别记作B1,B2
            //过点 B1 引 O1O2 的垂线，垂足为 M
            // O1O2 的斜率为k
            // 切点
            PointF[] CutPoints = new PointF[2];
            // 切線
            Line l1 ;
            Line l2 ;

            float deltaX = circle2.center.X - circle1.center.X;
            float deltaY = circle2.center.Y - circle1.center.Y;
            // O1 与 O2 距离
            float distance = 0;
            // P 与 O2 距离
            float lengthA;
            // P 与 B1 或 B2点的距离
            float lengthB;
            // B1 到 O1O2的距离
            float lengthC;
            // O2 到 M 的距离
            float lengthD;
            //用于记录 M 点的坐标
            PointF M = new PointF();
            //  O1O2 直线方程的斜率k
            float k;
            
            //  O1O2 直线方程中的常数
            float b;
            distance = (float)(Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2)));
            lengthA = distance * circle1.radius / (circle2.radius - circle1.radius);
            lengthB = (float)(Math.Sqrt(Math.Pow(lengthA, 2) - Math.Pow(circle1.radius, 2)));
            lengthC = lengthB * circle1.radius / lengthA;
            lengthD = circle1.radius * circle1.radius / lengthA;
            M.X = circle1.center.X + lengthD * -deltaX / distance;
            M.Y = circle1.center.Y + lengthD * -deltaY / distance;
            k = (circle1.center.Y - circle2.center.Y) / (circle1.center.X - circle2.center.X);
            b = circle1.center.Y - k * circle1.center.X;

            float Y1_1 = 0;
            float X1_1 = 0;
            Y1_1 = (float)((k * M.X + M.Y * k * k + b + Math.Abs(lengthC) * Math.Sqrt(k * k + 1)) / (k * k + 1));
            X1_1 = M.X - (Y1_1 - M.Y) * k;
            CutPoints[0].X = X1_1;
            CutPoints[0].Y = Y1_1;

            // normalized vector from circle2 to b1
            Vector2 circle1_b1 = Vector2.Normalize(new Vector2(X1_1 - circle1.center.X, Y1_1 - circle1.center.Y));
            float X1_2 = circle2.center.X + circle2.radius * circle1_b1.X;
            float Y1_2 = circle2.center.Y + circle2.radius * circle1_b1.Y;

            l1 = new Line(new PointF((float)X1_1, (float)Y1_1), new PointF((float)X1_2, (float)Y1_2));

            float Y2_1 = 0;
            float X2_1 = 0;
            Y2_1 = (float)((k * M.X + M.Y * k * k + b - Math.Abs(lengthC) * Math.Sqrt(k * k + 1)) / (k * k + 1));
            X2_1 = M.X - (Y2_1 - M.Y) * k;
            CutPoints[1].X = X2_1;
            CutPoints[1].Y = Y2_1;

            // normalized vector from circle2 to b2
            Vector2 circle1_b2 = Vector2.Normalize(new Vector2(X2_1 - circle1.center.X, Y2_1 - circle1.center.Y));
            float X2_2 = circle2.center.X + circle2.radius * circle1_b2.X;
            float Y2_2 = circle2.center.Y + circle2.radius * circle1_b2.Y;
            
            l2 = new Line(new PointF((float)X2_1, (float)Y2_1), new PointF((float)X2_2, (float)Y2_2));

            return (l1, l2);
        } 
    
        /// <summary>
        /// 計算兩條直線的交點
        /// </summary>
        /// <param name="lineFirstStar">L1的點1坐標</param>
        /// <param name="lineFirstEnd">L1的點2坐標</param>
        /// <param name="lineSecondStar">L2的點1坐標</param>
        /// <param name="lineSecondEnd">L2的點2坐標</param>
        /// <returns>PointF交點坐標</returns>
        public static PointF GetIntersection(PointF lineFirstStar, PointF lineFirstEnd, PointF lineSecondStar, PointF lineSecondEnd)
        {
            /*
             * L1，L2都存在斜率的情況：
             * 直線方程L1: ( y - y1 ) / ( y2 - y1 ) = ( x - x1 ) / ( x2 - x1 ) 
             * => y = [ ( y2 - y1 ) / ( x2 - x1 ) ]( x - x1 ) + y1
             * 令 a = ( y2 - y1 ) / ( x2 - x1 )
             * 有 y = a * x - a * x1 + y1   .........1
             * 直線方程L2: ( y - y3 ) / ( y4 - y3 ) = ( x - x3 ) / ( x4 - x3 )
             * 令 b = ( y4 - y3 ) / ( x4 - x3 )
             * 有 y = b * x - b * x3 + y3 ..........2
             * 
             * 如果 a = b，則兩直線平等，否則， 聯解方程 1,2，得:
             * x = ( a * x1 - b * x3 - y1 + y3 ) / ( a - b )
             * y = a * x - a * x1 + y1
             * 
             * L1存在斜率, L2平行Y軸的情況：
             * x = x3
             * y = a * x3 - a * x1 + y1
             * 
             * L1 平行Y軸，L2存在斜率的情況：
             * x = x1
             * y = b * x - b * x3 + y3
             * 
             * L1與L2都平行Y軸的情況：
             * 如果 x1 = x3，那麼L1與L2重合，否則平等
             * 
            */
            float a = 0, b = 0;
            int state = 0;
            if (lineFirstStar.X != lineFirstEnd.X)
            {
                a = (lineFirstEnd.Y - lineFirstStar.Y) / (lineFirstEnd.X - lineFirstStar.X);
                state |= 1;
            }
            if (lineSecondStar.X != lineSecondEnd.X)
            {
                b = (lineSecondEnd.Y - lineSecondStar.Y) / (lineSecondEnd.X - lineSecondStar.X);
                state |= 2;
            }
            switch (state)
            {
                case 0: //L1與L2都平行Y軸
                    {
                        if (lineFirstStar.X == lineSecondStar.X)
                        {
                            //throw new Exception("兩條直線互相重合，且平行於Y軸，無法計算交點。");
                            return new PointF(float.NaN, float.NaN);
                        }
                        else
                        {
                            //throw new Exception("兩條直線互相平行，且平行於Y軸，無法計算交點。");
                            return new PointF(float.NaN, float.NaN);
                        }
                    }
                case 1: //L1存在斜率, L2平行Y軸
                    {
                        float x = lineSecondStar.X;
                        float y = (lineFirstStar.X - x) * (-a) + lineFirstStar.Y;
                        return new PointF(x, y);
                    }
                case 2: //L1 平行Y軸，L2存在斜率
                    {
                        float x = lineFirstStar.X;
                        //網上有相似代碼的，這一處是錯誤的。你可以對比case 1 的邏輯 進行分析
                            //源code:lineSecondStar * x + lineSecondStar * lineSecondStar.X + p3.Y;
                        float y = (lineSecondStar.X - x) * (-b) + lineSecondStar.Y;
                        return new PointF(x, y);
                    }
                case 3: //L1，L2都存在斜率
                    {
                        if (a == b)
                        {
                            // throw new Exception("兩條直線平行或重合，無法計算交點。");
                            return new PointF(float.NaN, float.NaN);
                        }
                        float x = (a * lineFirstStar.X - b * lineSecondStar.X - lineFirstStar.Y + lineSecondStar.Y) / (a - b);
                        float y = a * x - a * lineFirstStar.X + lineFirstStar.Y;
                        return new PointF(x, y);
                    }
            }
            // throw new Exception("不可能發生的情況");
            return new PointF(float.NaN, float.NaN);
        }


        /// <summary>  
        /// 根据余弦定理求两个线段夹角  
        /// </summary>  
        /// <param name="o">端点</param>  
        /// <param name="s">start点</param>  
        /// <param name="e">end点</param>  
        /// <returns>Angle</returns>  
       public static double Angle(PointF o, PointF s, PointF e)  
        {  
            double cosfi = 0, fi = 0, norm = 0;  
            double dsx = s.X - o.X;  
            double dsy = s.Y - o.Y;  
            double dex = e.X - o.X;  
            double dey = e.Y - o.Y;  
        
            cosfi = dsx * dex + dsy * dey;  
            norm = (dsx * dsx + dsy * dsy) * (dex * dex + dey * dey);  
            cosfi /= Math.Sqrt(norm);  
        
            if (cosfi >= 1.0) return 0;  
            if (cosfi <= -1.0) return Math.PI;  
            fi = Math.Acos(cosfi);  
        
            if (180 * fi / Math.PI < 180)  
            {  
                return 180 * fi / Math.PI;  
            }  
            else  
            {  
                return 360 - 180 * fi / Math.PI;  
            }  
        }

        /// <summary>  
        /// 判斷兩直線所構成之夾角是否為銳角，其中判定方向為兩線夾護衛艦方向的夾角
        /// </summary>
        /// <param name="l1">線段1</param>  
        /// <param name="l2">線段2</param>  
        /// <param name="war_ship">護衛艦</param>  
        /// <returns>True:銳角;False:鈍角</returns>  
        private static bool IsAcuteAngle(Line l1, Line l2, Circle war_ship)
        {
            PointF intersectpoint = GetIntersection(l1.PointA, l1.PointB, l2.PointA, l2.PointB);
            PointF cutpoint_warship_l1;
            PointF cutpoint_warship_l2;
            
            if (Distance(war_ship.center, l1.PointA) == war_ship.radius) cutpoint_warship_l1 = l1.PointA;
            else cutpoint_warship_l1 = l1.PointB;
            
            if (Distance(war_ship.center, l2.PointA) == war_ship.radius) cutpoint_warship_l2 = l2.PointA;
            else cutpoint_warship_l2 = l2.PointB;
            
            double angle = Angle(intersectpoint, cutpoint_warship_l1, cutpoint_warship_l2);

            if (angle < 90) return true;
            else return false;
 

        }

        /// <summary>  
        /// 在tagent_lines中，兩切線夾角為銳角的切線中插入新的切線
        /// </summary>
        /// <param name="l1">線段1</param>
        /// <param name="l2">線段2</param>
        /// <param name="war_ship">護衛艦</param>
        /// <returns>Line l 切線</returns>
        private static Line NewCutLine(Line l1, Line l2, Circle war_ship)
        {   
            // 兩切線交點
            PointF intersectpoint = GetIntersection(l1.PointA, l1.PointB, l2.PointA, l2.PointB);
            // 角平分線與威脅圓最近的交點
            PointF closestpointoncircle = ClosestIntersection(war_ship.center.X , war_ship.center.Y, war_ship.radius, 
                                                                intersectpoint, war_ship.center);
            
            // 兩切線交點到護衛艦圓心之單位向量
            Vector2 bisector_vec = Vector2.Normalize(new Vector2(war_ship.center.X - intersectpoint.X, war_ship.center.Y - intersectpoint.Y));
            // 兩切線交點到護衛艦圓心之單位向量的左邊法向量
            Vector2 bisector_normal_vec = new Vector2(-bisector_vec.Y, bisector_vec.X);

            // 法向量的另外一點
            PointF new_point_on_normal = new PointF(closestpointoncircle.X + 10 * bisector_normal_vec.X, 
                                                    closestpointoncircle.Y + 10 * bisector_normal_vec.Y);

            Line newcutline = new Line(closestpointoncircle, new_point_on_normal);

            return newcutline;

        }

    }

}