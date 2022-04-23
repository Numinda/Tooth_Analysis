using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ZedGraph;
using CSML;

using System.Windows.Forms;

namespace TripleArc
{
    class Triple_Arc
    {
        //flex spline tooth
        //radius
        const int r1 = 24;  //dedendum1 radius
        const int r2 = 56;  //dedendum2 radius
        const int r3 = 98;  //addendum1 radius
        const int r4 = 26;  //addendum2 radius
        //centers
        const double x1 = 48.54384469;  //dedendum1 center
        const double y1 = 23.49660389;
        const double x2 = 78;  //dedendum2 center
        const double y2 = 36;
        const double x3 = -75.62486374;  //addendum1 center
        const double y3 = 25.25750304;
        const double x4 = -6.078204247;  //addendum2 center
        const double y4 = 43.89247429;
        const double x5 = 8.834783098;  //upper line
        const double y5 = 65.19042744;

        const double w = 88.75257686;//=m*pi
        const double t = 6;
        //angle limits of the flex spline tooth
        const int FsDedendum1Low = 10;  //dedendum1
        const int FsDedendum1High = 67;
        const int FsDedendum2Low = 67; //dedendum2
        const int FsDedendum2High = 86;
        const int FsAddendum1Low = 4; //addendum1
        const int FsAddendum1High = 15;
        const int FsAddendum2Low = 15; //addendum2
        const int FsAddendum2High = 55;

        //angle limits of the circular spline tooth
        const int CsAddendum1Low = 40;  //addendum1
        const int CsAddendum1High = 200;
        const int CsAddendum2Low = 120;  //addendum2
        const int CsAddendum2High = 180;
        const int CsDedendum1Low = 5;  //dedendum1
        const int CsDedendum1High = 20;
        const int CsDedendum2Low = 10;  //dedendum2
        const int CsDedendum2High = 77;

        //Calculating the angle limits of the circular spline curves
        //public CalculateAngleLimits ()


        //curves of the flex spline
        public PointPairList CreateFsDedendum1(double module)
        {
            PointPairList pointPairList = new PointPairList();

            for (int theta = FsDedendum1Low; theta <= FsDedendum1High; theta++)
            {
                double th = theta * Math.PI / 180;
                double x = (-r1 * Math.Sin(th) + x1) * Math.PI * module / w;
                double y = (y1 - r1 * Math.Cos(th)) * Math.PI * module / w;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }
        public PointPairList CreateFsDedendum2(double module)
        {
            PointPairList pointPairList = new PointPairList();

            for (int theta = FsDedendum2Low; theta <= FsDedendum2High; theta++)
            {
                double th = theta * Math.PI / 180;
                double x = (-r2 * Math.Sin(th) + x2) * Math.PI * module / w;
                double y = (y2 - r2 * Math.Cos(th)) * Math.PI * module / w;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }
        public PointPairList CreateFsAddendum1(double module)
        {
            PointPairList pointPairList = new PointPairList();

            for (int theta = FsAddendum1Low; theta <= FsAddendum1High; theta++)
            {
                double th = theta * Math.PI / 180;
                double x = (r3 * Math.Cos(th) + x3) * Math.PI * module / w;
                double y = (y3 + r3 * Math.Sin(th)) * Math.PI * module / w;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }

        public PointPairList CreateFsAddendum2(double module)
        {
            PointPairList pointPairList = new PointPairList();

            for (int theta = FsAddendum2Low; theta <= FsAddendum2High; theta++)
            {
                double th = theta * Math.PI / 180;
                double x = (x4 + r4 * (Math.Cos(th))) * Math.PI * module / w;
                double y = (y4 + r4 * Math.Sin(th)) * Math.PI * module / w;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }
        public PointPairList CreateFsUpper(double module)
        {
            PointPairList pointPairList = new PointPairList();

            double x = x5 * Math.PI * module / w;
            double y = y5 * Math.PI * module / w;
            PointPair pointPair1 = new PointPair(x, y);
            pointPairList.Add(pointPair1);

            double y1 = y5 * Math.PI * module / w;
            double x1 = 0;
            PointPair pointPair2 = new PointPair(x1, y1);
            pointPairList.Add(pointPair2);

            return pointPairList;
        }

        //V ring gear tooth
        public PointPairList CreateVeeCsFlank(double module)
        {
            PointPairList pointPairList = new PointPairList();

            double x = -Math.PI * module / 2;
            double y = 0;
            PointPair pointPair1 = new PointPair(x, y);
            pointPairList.Add(pointPair1);

            double y1 = y5 * Math.PI * module / w;
            double x1 = 0;
            PointPair pointPair2 = new PointPair(x1, y1);
            pointPairList.Add(pointPair2);

            return pointPairList;

        }

        public PointPairList DeformedFsToothCentreLines(PointPairList tooth, double deflection, double module, double angle, int teeth)
        {

            PointPairList pointPairList = new PointPairList();
            int pointcount = tooth.Count();

            Matrix pointin = new Matrix(4, 1);
            Matrix pointout = new Matrix(4, 1);

            Matrix translation1 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation2 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation3 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation4 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");

            Matrix rotation = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix Rotation = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");

            double pitchdiameter = module * teeth;


            double u, v, shi;
            double movementofcenter = (t / 2) * (Math.PI * module / w);

            double theta =  Math.PI * angle / 180;
            u = deflection * module * Math.Cos(2*theta) - 1 * module;
            v = - 0.5 * deflection * module * Math.Sin(2*theta);
            shi =  deflection * module * (( Math.Sin(2*theta) / pitchdiameter) - (2 * Math.Sin(2*theta) / (pitchdiameter / 2 + (u+ module))  ));
            double to =  2 * v / pitchdiameter;
           

            //shift center to fs neutral line
            translation1[2, 4].Re = -movementofcenter;

            //rotate tooth center line
            rotation[1, 1].Re = Math.Cos(shi);
            rotation[1, 2].Re = -Math.Sin(shi);
            rotation[2, 1].Re = Math.Sin(shi);
            rotation[2, 2].Re = Math.Cos(shi);

            //radial shift
            translation2[1, 4].Re = 0;
            translation2[2, 4].Re =  u;

            //restore oginal centre on the pitch circle
            translation3[2, 4].Re = movementofcenter;

            //move tooth to pitch circle
            translation4[2, 4].Re = pitchdiameter / 2;

            //rotate tooth to correct angular location
            Rotation[1, 1].Re = Math.Cos(theta + to);
            Rotation[1, 2].Re = Math.Sin(theta + to);
            Rotation[2, 1].Re = -Math.Sin(theta + to);
            Rotation[2, 2].Re = Math.Cos(theta + to);

            Matrix M = Rotation * translation4 * translation3 * translation2 * rotation * translation1;

            for (int j = 0; j <= pointcount - 1; j++)
            {
                pointin[1, 1].Re = tooth[j].X;
                pointin[2, 1].Re = tooth[j].Y;
                pointin[3, 1].Re = 0;
                pointin[4, 1].Re = 1;
                
                pointout =  M * pointin;
                PointPair pointpair = new PointPair(pointout[1, 1].Re, pointout[2, 1].Re);
                pointPairList.Add(pointpair);
            }

            return pointPairList;

        }


        public PointPairList DeformedFsReference(PointPairList tooth, double deflection, double module, int fsteethcount, int teethnumber )
        {

            PointPairList pointPairList = new PointPairList();
            int pointcount = tooth.Count();

            Matrix pointin = new Matrix(4, 1);
            Matrix pointout = new Matrix(4, 1);

            Matrix translation1 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation2 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation3 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation4 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation5 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");

            Matrix rotation = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix RotationF = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");
            Matrix RotationB = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");

            double pitchdiameter = module * fsteethcount;


            double u, v, shi;
            double movementofcenter = (t / 2) * (Math.PI * module / w);

            double theta = 2*Math.PI * teethnumber / fsteethcount;
            double alpha = - 2* Math.PI * teethnumber / (fsteethcount+2);

            //if ((teethnumber < 20) & (teethnumber > 30))
            //{
            //    alpha = -2 * Math.PI * teethnumber / (fsteethcount + 2);
            //}
            //else
            //{
            //  //  alpha = -2 * Math.PI * teethnumber / (fsteethcount);
            //}

            u = deflection * module * Math.Cos(2 * theta) - 1 * module;
            v = - 0.5 * deflection * module * Math.Sin(2 * theta);
            shi = deflection * module * ((Math.Sin(2 * theta) / pitchdiameter) - (2 * Math.Sin(2 * theta) / (pitchdiameter / 2 + (u + module))));
            double to =  2 * v / pitchdiameter;


            //shift center to fs neutral line
            translation1[2, 4].Re = -movementofcenter;

            //rotate tooth center line
            rotation[1, 1].Re = Math.Cos(shi);
            rotation[1, 2].Re = -Math.Sin(shi);
            rotation[2, 1].Re = Math.Sin(shi);
            rotation[2, 2].Re = Math.Cos(shi);

            //radial shift
            translation2[1, 4].Re = 0;
            translation2[2, 4].Re = u;

            //restore oginal centre on the pitch circle
            translation3[2, 4].Re = movementofcenter;

            //move tooth to pitch circle
            translation4[2, 4].Re = pitchdiameter / 2;

            //rotate tooth to correct angular location
            RotationF[1, 1].Re = Math.Cos(theta + to);
            RotationF[1, 2].Re = Math.Sin(theta + to);
            RotationF[2, 1].Re = -Math.Sin(theta + to);
            RotationF[2, 2].Re = Math.Cos(theta + to);

            //Rotate back to refrence
            RotationB[1, 1].Re = Math.Cos(alpha);
            RotationB[1, 2].Re = Math.Sin(alpha);
            RotationB[2, 1].Re = -Math.Sin(alpha);
            RotationB[2, 2].Re = Math.Cos(alpha);

            //move origin
            translation5[2, 4].Re = - pitchdiameter / 2;


            Matrix M = translation5 * RotationB * RotationF * translation4 * translation3 * translation2 * rotation * translation1;
            for (int j = 0; j <= pointcount - 1; j++)
            {
                pointin[1, 1].Re = tooth[j].X;
                pointin[2, 1].Re = tooth[j].Y;
                pointin[3, 1].Re = 0;
                pointin[4, 1].Re = 1;

                pointout =  M * pointin;
                PointPair pointpair = new PointPair(pointout[1, 1].Re, pointout[2, 1].Re);
                pointPairList.Add(pointpair);
            }

            return pointPairList;

        }

        //half of the tooth
        public PointPairList CreateHalfTooth(PointPairList one, PointPairList two, PointPairList three, PointPairList four, PointPairList five)
        {
            PointPairList pointPairList = new PointPairList();

            pointPairList.Add(one);
            pointPairList.Add(two);
            pointPairList.Add(three);
            pointPairList.Add(four);
            pointPairList.Add(five);
            return pointPairList;
        }
        //Full tooth
        public PointPairList CreateFullTooth(PointPairList one, PointPairList two)
        {
            PointPairList pointPairList = new PointPairList();
            pointPairList.Add(one);
            pointPairList.Add(two);
            return pointPairList;
        }
        //mirror curves
        public PointPairList Mirror(PointPairList rightside)
        {
            PointPairList mirror = new PointPairList();
            int pointcount = rightside.Count();
            Matrix point = new Matrix(4, 1);
            Matrix newpoint = new Matrix(4, 1);
            Matrix rot = new Matrix("-1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1");

            for (int i = pointcount - 1; i >= 0; i--)
            {
                point[1, 1].Re = rightside[i].X;
                point[2, 1].Re = rightside[i].Y;
                point[3, 1].Re = 0;
                point[4, 1].Re = 1;
                newpoint = rot * point;
                PointPair pointpairmirror = new PointPair(newpoint[1, 1].Re, newpoint[2, 1].Re);
                mirror.Add(pointpairmirror);
            }
            return mirror;

        }


        //motion of the tooth on the base curve with camangle
        public PointPairList FSTeethBaseCurveMotion(PointPairList tooth, double k, double module, double camangle)
        {
            PointPairList pointPairList = new PointPairList();
            int pointcount = tooth.Count();

            double ca = 2 * camangle * Math.PI / 180;
            double yshift = k * module * Math.Cos(ca) - k * module;
            double xshift = 0.5 * module * (ca - k * Math.Sin(ca));
            for (int i = pointcount - 1; i >= 0; i--)
            {
                double x = tooth[i].X + xshift;
                double y = tooth[i].Y + yshift;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }

            return pointPairList;
        }

        //motion of the tooth considering u,v and shi
        public PointPairList FSTeethNewMotion(PointPairList tooth, double deflection, double module, double angle, double fsteethcount)
        {

            PointPairList pointPairList = new PointPairList();
            int pointcount = tooth.Count();

            Matrix pointin = new Matrix(4, 1);
            Matrix pointout = new Matrix(4, 1);

            Matrix translation1 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation2 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation3 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation4 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix translation5 = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");

            Matrix rotation = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,0; 0, 0, 1,0; 0,0,0,1");
            Matrix RotationF = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");
            Matrix RotationB = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");

            double pitchdiameter = module * fsteethcount;


            double u, v, shi;
            double movementofcenter = (t / 2) * (Math.PI * module / w);

            double theta =  Math.PI * angle / 180;
            double alpha = -theta * fsteethcount / (fsteethcount + 2);

            u = deflection * module * Math.Cos(2 * theta) - 1 * module;
            v = - 0.5 * deflection * module * Math.Sin(2 * theta);
            shi = deflection * module * ((Math.Sin(2 * theta) / pitchdiameter) - (2 * Math.Sin(2 * theta) / (pitchdiameter / 2 + (u + module))));
            double to = 2 * v / pitchdiameter;


            //shift center to fs neutral line
            translation1[2, 4].Re = -movementofcenter;

            //rotate tooth center line
            rotation[1, 1].Re = Math.Cos(shi);
            rotation[1, 2].Re = -Math.Sin(shi);
            rotation[2, 1].Re = Math.Sin(shi);
            rotation[2, 2].Re = Math.Cos(shi);

            //radial shift
            translation2[1, 4].Re = 0;
            translation2[2, 4].Re = u;

            //restore oginal centre on the pitch circle
            translation3[2, 4].Re = movementofcenter;

            //move tooth to pitch circle
            translation4[2, 4].Re = pitchdiameter / 2;

            //rotate tooth to correct angular location
            RotationF[1, 1].Re = Math.Cos(theta + to);
            RotationF[1, 2].Re = Math.Sin(theta + to);
            RotationF[2, 1].Re = -Math.Sin(theta + to);
            RotationF[2, 2].Re = Math.Cos(theta + to);

            //Rotate back to refrence
            RotationB[1, 1].Re = Math.Cos(alpha);
            RotationB[1, 2].Re = Math.Sin(alpha);
            RotationB[2, 1].Re = -Math.Sin(alpha);
            RotationB[2, 2].Re = Math.Cos(alpha);

            //move origin
            translation5[2, 4].Re = -pitchdiameter / 2;

            Matrix M = translation5 * RotationB * RotationF * translation4 * translation3 * translation2 * rotation * translation1;

            for (int j = 0; j <= pointcount - 1; j++)
            {
                pointin[1, 1].Re = tooth[j].X;
                pointin[2, 1].Re = tooth[j].Y;
                pointin[3, 1].Re = 0;
                pointin[4, 1].Re = 1;

                pointout = M * pointin;
                PointPair pointpair = new PointPair(pointout[1, 1].Re, pointout[2, 1].Re);
                pointPairList.Add(pointpair);
            }

            return pointPairList;
        }

        //Tooth center line to check wheather the new path of the tooth is correct
        public PointPairList CreateFSToothCenterLine(double module)
        {
            PointPairList pointPairList = new PointPairList();
            double x1 = 0;
            double y1 = 0;
            PointPair pointPair1 = new PointPair(x1, y1);
            pointPairList.Add(pointPair1);
            double x2 = 0;
            double y2 =  y5 * Math.PI * module / w;
            PointPair pointPair2 = new PointPair(x2, y2);
            pointPairList.Add(pointPair2);
            pointPairList.Add(pointPair1);

            return pointPairList;
        }

        //Tooth top center check with Base Curve
        public PointPairList CreateFSToothTopCenter(double module)
        {
            PointPairList pointPairList = new PointPairList();
            double x1 = 0.1* Math.PI * module / w;
            double y1 = y5 * Math.PI * module / w;
            PointPair pointPair1 = new PointPair(x1, y1);
            pointPairList.Add(pointPair1);
             x1 = -0.1* Math.PI * module / w;
            PointPair pointPair2 = new PointPair(x1, y1);
            pointPairList.Add(pointPair2);

            return pointPairList;
        }


        //deformed tooth moving

        //base curve - the paths of the points of the flex spline
        public PointPairList CreatepathTopCenter(double k, double module)
        {
            PointPairList pointPairList = new PointPairList();
            double yshift = y5 * Math.PI * module / w - k * module;

            for (int theta = -90; theta <= 90; theta++)
            {
                double th = 2 * Math.PI * theta / 180;
                double x = 0.5 * module * (th - k * Math.Sin(th));
                double y = k * module * Math.Cos(th) + yshift;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }
        //circular spline
        public PointPairList CreateCsAddendum1(double module, double deflection)
        {
            PointPairList pointPairList = new PointPairList();
            //if (deflection == 0.9)
            {
                for (double theta = CsAddendum1Low; theta <= CsAddendum1High; theta++)
                {
                    double th = theta * Math.PI / 180;
                    double s = Math.Sin(th);
                    double c = Math.Cos(th);

                    double alpha = 90 * Math.PI / 180 - Math.Atan(((2 * deflection * s)) / (1 - deflection * c));
                    double x = (0.5 * module * (th - deflection * s)) + (r4 * Math.Cos(alpha) + x4) * Math.PI * module / w;
                    double y = (deflection * module * c) - module + (r4 * Math.Sin(alpha) + y4) * Math.PI * module / w;

                    PointPair pointPair = new PointPair(x, y);
                    pointPairList.Add(pointPair);
                }
            }

            return pointPairList;

        }
        public PointPairList CreateCsAddendum2(double module, double deflection)
        {
            PointPairList pointPairList = new PointPairList();
            //    if (deflection == 0.9)
            {
                for (double theta = CsAddendum2Low; theta <= CsAddendum2High; theta++)
                {

                    double th = theta * Math.PI / 180;
                    double s = Math.Sin(th);
                    double c = Math.Cos(th);

                    double alpha = FsAddendum2High * Math.PI / 180;

                    double x = (0.5 * module * (th - deflection * s)) + (r4 * Math.Cos(alpha) + x4) * Math.PI * module / w;
                    double y = (deflection * module * c) - module + (r4 * Math.Sin(alpha) + y4) * Math.PI * module / w;

                    PointPair pointPair = new PointPair(x, y);
                    pointPairList.Add(pointPair);


                }
            }
            return pointPairList;
        }



        public PointPairList CreateCsDedendum1(double module, double deflection)
        {
            PointPairList pointPairList = new PointPairList();

            for (int theta = CsDedendum1Low; theta <= CsDedendum1High; theta++)
            {
                double th = theta * Math.PI / 180;
                double x = (r3 * Math.Cos(th) + x3) * Math.PI * module / w;
                double y = (y3 + r3 * Math.Sin(th)) * Math.PI * module / w + (deflection - 1) * module;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }

        public PointPairList CreateCsDedendum2(double module, double deflection)
        {
            PointPairList pointPairList = new PointPairList();

            for (double theta = CsDedendum2Low; theta <= CsDedendum2High; theta = theta + 0.01)
            {
                double th = theta * Math.PI / 180;
                double x = (x4 + r4 * (Math.Cos(th))) * Math.PI * module / w;
                double y = (y4 + r4 * Math.Sin(th)) * Math.PI * module / w + (deflection - 1) * module;
                PointPair pointPair = new PointPair(x, y);
                pointPairList.Add(pointPair);
            }
            return pointPairList;
        }


        //translate curves
        public PointPairList Translate(PointPairList teeth, double offset)
        {
            PointPairList translate = new PointPairList();
            int pointcount = teeth.Count();
            Matrix point = new Matrix(4, 1);
            Matrix newpoint = new Matrix(4, 1);
            Matrix T = new Matrix("1,0,0,0;0,1,0,1;0,0,1,0;0,0,0,1");
            T[2, 4].Re = offset;
            for (int i = pointcount - 1; i >= 0; i--)
            {
                point[1, 1].Re = teeth[i].X;
                point[2, 1].Re = teeth[i].Y;
                point[3, 1].Re = 0;
                point[4, 1].Re = 1;
                newpoint = T * point;
                PointPair pointpairtranslate = new PointPair(newpoint[1, 1].Re, newpoint[2, 1].Re);
                translate.Add(pointpairtranslate);
            }
            return translate;

        }
        //rotation of tooth
        public PointPairList Rotate(PointPairList teeth, int teethcount)
        {
            PointPairList pointPairList = new PointPairList();
            int pointcount = teeth.Count();
            Matrix point = new Matrix(4, 1);
            Matrix tpoint = new Matrix(4, 1);
            Matrix R = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");
            double theta = 0;

            for (int j = 0; j < teethcount; j++)
            {
                theta += 2* Math.PI / teethcount;

                R[1, 1].Re = Math.Cos(theta);
                R[1, 2].Re = Math.Sin(theta);
                R[2, 1].Re = -Math.Sin(theta);
                R[2, 2].Re = Math.Cos(theta);

                for (int i = pointcount - 1; i >= 0; i--)
                {
                    point[1, 1].Re = teeth[i].X;
                    point[2, 1].Re = teeth[i].Y;
                    point[3, 1].Re = 0;
                    point[4, 1].Re = 1;
                    tpoint = R * point;
                    PointPair pointpair = new PointPair(tpoint[1, 1].Re, tpoint[2, 1].Re);
                    pointPairList.Add(pointpair);

                }
                theta += Math.PI * teethcount;
            }
            return pointPairList;
        }
        //deforming the flex spline
        public PointPairList DeformGear(PointPairList flex, double deflection, double module, double angle,  int teeth)
        {
            PointPairList pointPairList = new PointPairList();
            int pointcount = flex.Count();
            Matrix point = new Matrix(4, 1);
            Matrix tpoint = new Matrix(4, 1);
            Matrix tpointr = new Matrix(4, 1);
            Matrix translation = new Matrix("1, 0 , 0 ,0; 0 , 1, 0,10; 0, 0, 1,0; 0,0,0,1");
            double u, v, shi;
            Matrix Rotation2 = new Matrix("1, 1,0,0; 1 , 1, 0,0; 0, 0, 1, 0; 0, 0, 0, 1");

            double pitchdiameter = module * teeth;

            for (int i = 0; i <= teeth; i++)
            {
                double theta0 = (2 * i * Math.PI / teeth) + Math.PI * angle / 180;
                Rotation2[1, 1].Re = Math.Cos(theta0);
                Rotation2[1, 2].Re = -Math.Sin(theta0);
                Rotation2[2, 1].Re = Math.Sin(theta0);
                Rotation2[2, 2].Re = Math.Cos(theta0);
                double theta = Math.PI * angle / 180;
                u = deflection * module * Math.Cos(2 * theta);
                v = -deflection * module * Math.Sin(2 * theta) / 2;
                shi = -3 * deflection * Math.Sin(2 * theta * Math.PI / 180) / (2 * Math.Sqrt(Math.Pow((pitchdiameter / 2 + u), 2) + Math.Pow(v, 2)));
                translation[1, 1].Re = Math.Cos(shi);
                translation[1, 2].Re = -Math.Sin(shi);
                translation[2, 1].Re = Math.Sin(shi);
                translation[2, 2].Re = Math.Cos(shi);
                translation[1, 4].Re = v;
                translation[2, 4].Re = pitchdiameter / 2 + u;
                for (int j = 0; j <= pointcount - 1; j++)
                {
                    point[1, 1].Re = flex[j].X;
                    point[2, 1].Re = flex[j].Y;
                    point[3, 1].Re = 0;
                    point[4, 1].Re = 1;
                    tpointr = translation * point;
                    tpoint = Rotation2 * tpointr;
                    PointPair pointpair = new PointPair(tpoint[1, 1].Re, tpoint[2, 1].Re);
                    pointPairList.Add(pointpair);
                }
            }
            return pointPairList;

        }
    }
}
