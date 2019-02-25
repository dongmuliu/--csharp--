using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Drawing.Imaging;
using System.Collections;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.CvEnum;
using Emgu.CV.Util;
using SharpGL;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Timers;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace _20180422TOFDetection
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            this.pictureBox1.MouseMove += new MouseEventHandler(DepthInfoPic1);
            this.pictureBox4.MouseMove += new MouseEventHandler(DepthInfoPic2);
            this.TestTimer.Elapsed += new ElapsedEventHandler(TestTimer_Elapsed);  //初始化，达到时间间隔时，触发委托事件
        }

        [DllImport("CreateDll.dll", EntryPoint = "caliprocess")]


        public extern static void caliprocess(int N, [MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] ushort[] img);
        //Socket 全局变量定义
        public Socket clintSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        IPAddress dfip;
        int dfport;
        bool iscollected = false;
        /////////////////////////////////////////////////////////////////////////////////////////////
        //深度相机基本参数
        int CameraDepth = 240; //相机距离地面的高度

        ///////////////////////////////////////////////// ///////////////////////////////////////////
        //深度相机采集数据全局变量
        byte[] result = new byte[153600];
        byte[] dis;//用于socket保存接收到的深度与光强信息的原始数据
        int[,] disp, disp1, disp2; //用作数据处理

        Bitmap sourceimg;
        ushort[] oneArry;
        ///////////////////////////////////////////////// ///////////////////////////////////////////
        //深度相机采处理过程全局变量
        int[,] backImgDepth = new int[252, 328];//背景图片的深度数据  252是height,328是width
        int[,] srcImgDepth = new int[252, 328];//目标物体图片的深度数据
        int[,] differImgData = new int[328, 252];//差分后图片的深度数据

        int[,] srcImgData = new int[252, 328]; //目标图片原始数据 用来渲染
        int[,] backImgData = new int[252, 328]; //背景图片原始数据 用来渲染


        //连通域处理过程中全局变量
        ushort[,] newsignall = new ushort[252, 328];
        int connectareanumber = 0;
        int max = 0;
        float p240;  //距离映射比
        float[] p240s = new float[100]; //所有连通域对应的映射比，p240s[0]对应连通域1...
        bool connectAreaFlag = false;//是否显示连通域的标记
        double actWidth, actHeight;//最终实际的长度和宽度


        //传递给SharpGL的测量数据，长宽高
        float aWidth, aHeight, aDepth;

        //SharpGL绘图过程绕着X,Y,Z轴的旋转度数，初始定义为0
        // X轴旋转度数
        private float _x = 0;
        // Y轴旋转度数
        private float _y = 0;
        //Z轴旋转度数
        private float _z = 0;



        /// <summary>
        /// 定时器触发自动采集图像
        /// </summary>

        //实例化Timer类
        private System.Timers.Timer TestTimer = new System.Timers.Timer(350);   //TestTimer.Elapsed ->> TestTimer_Elapsed

        private delegate void Test();//使用委托，来加载触发定时器的事件
        void TestTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            this.BeginInvoke(new Test(AutoDetection), new object[] { });
        }



        private void AutoDetection()
        {
            getdistanceandamplitudesorted(out dis);               //输出一维数组，按照字节存储
            disp = datatranshalf(dis);                            //一维数组转换为二维数组，字数据转换，数组大小为[320,240] （对应240*320）           
            disp1 = displace(disp);                               //翻转[h=240,w=320]   [240,320] （对应320*240）

            oneArry = srcDepthData(disp1);                          //二维转换为一维数组，类型为ushort
            caliprocess(oneArry.Length, oneArry);                 //调用杭州VR程序，在上位机进行数据DRNU处理，输出oneArry为一维数组

            disp2 = datatranshalf1to2(oneArry);                     //一维数组转换为二维数组，大小为[240,320]

            sourceimg = drawquaterpic(disp2);                     //深度数据转rgb图像

            pictureBox1.Image = cvInitUndistortMap(sourceimg);    //畸变校正并显示
            iscollected = true;
         
            
        }

        /// <summary>
        /// 下位机需要的是1维数组，大小以字为单位，这里转换为【240，320】大小
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public ushort[] srcDepthData(int[,] source)
        {
            int h = source.GetLength(0);//240
            int w = source.GetLength(1);//320
            ushort[] srcInputDepthData = new ushort[w * h];

            for (int i = 0; i < h; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    srcInputDepthData[j + 320 * i] = (ushort)source[i, j];
                }
            }
            return srcInputDepthData;
        }

        /// <summary>
        /// 触发事件
        /// 自动采集图像的触发事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button14_Click(object sender, EventArgs e)
        {
            if (iscollected)
            {
                label5.Text = "实时图像";
                TestTimer.Enabled = true;
            }
            else
            {
                MessageBox.Show("相机未连接！", "Error");
            }
            
        }

        public Bitmap cvInitUndistortMap(Bitmap src)
        {
            int width = 320;
            int height = 240;
            Matrix<float> mapx = new Matrix<float>(240, 320);
            Matrix<float> mapy = new Matrix<float>(240, 320);

            //相机内参矩阵
            Matrix<double> intrMatrix = new Matrix<double>(3, 3);
            intrMatrix.Data[0, 0] = 196.24;
            intrMatrix.Data[0, 1] = 0;
            intrMatrix.Data[1, 1] = 196.91;
            intrMatrix.Data[0, 2] = 173.11;
            intrMatrix.Data[1, 2] = 133;
            intrMatrix.Data[2, 2] = 1;

            //径向畸变矩阵
            Matrix<double> disMatrix = new Matrix<double>(5, 1);
            disMatrix.Data[0, 0] = -0.3;
            disMatrix.Data[1, 0] = 0.11;
            disMatrix.Data[2, 0] = 0;
            disMatrix.Data[3, 0] = 0;
            disMatrix.Data[4, 0] = 0;

            //实例化相机内参
            IntrinsicCameraParameters intrinsicMatrix = new IntrinsicCameraParameters();
            intrinsicMatrix.IntrinsicMatrix = intrMatrix;
            intrinsicMatrix.DistortionCoeffs = disMatrix;

            //畸变校正，得到校正矩阵mapx、mapy
            intrinsicMatrix.InitUndistortMap(width, height, out mapx, out mapy);  

            // Bitmap b = (Bitmap)pictureBox1.Image.Clone();
            Image<Bgr, byte> image = new Image<Bgr, byte>(src);
            Mat dst = image.Mat;   //Image转Mat,向Remap提供数据

            //重映射Remap：把原图像某位置的像素通过mapx、mapy转换到新的指定位置
            CvInvoke.Remap(dst, dst, mapx, mapy, Inter.Linear);
            //CvInvoke.Imshow("1", dst);

            Image<Bgr, byte> _image = dst.ToImage<Bgr, byte>();  //Mat转Image
            Bitmap dstbitmap = _image.ToBitmap();                // Image转Bitmap
            return dstbitmap;
        }


        /// <summary>
        /// 功能函数
        /// 点击图片时，显示对应像素点位置的深度信息
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        private void DepthInfoPic1(object sender, MouseEventArgs e)
        {
            if (iscollected)
            {
                if (pictureBox1.Image != null)
                {
                    // label7.Text = string.Format("Position:({0},{1})", e.X, e.Y);
                    label3.Text = string.Format("Position:({0},{1})  Depth Value:{2:f2}LSB", e.X, e.Y, disp2[e.Y, e.X]);

                }
            }
        }

        /// <summary>
        /// 触发事件
        /// 打开图片，初始化图片尺寸
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1openImage_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofg = new OpenFileDialog();
            ofg.InitialDirectory = @"D:\win\new_client191\4.25\1.51";
            ofg.Filter = @"bmp file(*.bmp)|*.*";
            ofg.Title = "打开图片";

            if (ofg.ShowDialog() == DialogResult.OK)
            {
                string pic_name = ofg.FileName;
                Bitmap image = (Bitmap)System.Drawing.Image.FromFile(pic_name);
                pictureBox1.Image = image;
                label5.Text = "当前打开图像";
            }
            else
            {
                MessageBox.Show("未加载图片！", "提示");
            }
            
        }

        /// <summary>
        /// 触发事件
        /// 获取背景图像的信息，深度数据进行处理，并显示
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button2backgroundImg_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofg = new OpenFileDialog();
            ofg.InitialDirectory = @"D:\win\new_client191\4.25\1.51";
            ofg.Filter = @"all files(*.*)|*.*";
            ofg.Title = "读入文件";
            int maxDistance = 0;//最大深度数据，用来转灰度图，和255对应，灰度值=depthdata/maxDis*255;
            if (ofg.ShowDialog() == DialogResult.OK)
            {
                string pic_name = ofg.FileName;
                string[] strRows = System.IO.File.ReadAllLines(pic_name);
                int j = 0;
                int[] opt;

                foreach (string str in strRows)
                {

                    string[] tmp = str.Split(new string[] { "," }, StringSplitOptions.None);

                    for (int i = 0; i < tmp.Length; i++)
                    {
                        string[] output = new string[328];
                        output[i] = tmp[i].Replace("NaN", "0");
                        opt = Array.ConvertAll<string, int>(output, value => Convert.ToInt32(value));
                        backImgDepth[j, i] = (opt[i]) * 1250 / (30000);
                        backImgData[j, i] = opt[i];

                        if (backImgDepth[j, i] > 30000)
                        {
                            backImgDepth[j, i] = 0;
                            backImgData[j, i] = 0;
                        }
                        else
                        {
                            if (backImgDepth[j, i] > maxDistance)
                            {
                                maxDistance = backImgDepth[j, i];
                            }
                        }
                    }
                    j = j + 1;

                }
                Bitmap b = (Bitmap)pictureBox1.Image.Clone();
                Bitmap b1 = (Bitmap)pictureBox1.Image.Clone();
                Bitmap b2 = (Bitmap)pictureBox1.Image.Clone();
                Bitmap b3 = (Bitmap)pictureBox1.Image.Clone();

                //文件1 的CSV文件对应的深度图像转换成0-255范围画出来
                //灰度图像
                //b1 = drawImg(b, backImgDepth, maxDistance);

                //RGB图像
                b1 = drawpic(backImgData);

                //深度值转换
                backImgDepth = DepthTrans(backImgDepth);

                // b1.RotateFlip(RotateFlipType.RotateNoneFlipY);
                pictureBox2.Image = b1;

                label8.Text = "背景图像";
            }
            else
            {
                MessageBox.Show("未加载图像！", "提示");
            }
            
        }

        /// <summary>
        /// 功能函数——显示彩色图像
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public Bitmap drawpic(int[,] source)
        {
            int h = source.GetLength(0);
            int w = source.GetLength(1);
            int[,] a = new int[h, w];//[240,320]

            for (int i = 0; i < h; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    a[i, j] = source[i, j];
                }
            }
            Bitmap dst = new Bitmap(w, h);

            BitmapData dstdata = dst.LockBits(new Rectangle(0, 0, w, h),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            double r, g, b, H;

            unsafe
            {
                byte* p0 = (byte*)dstdata.Scan0;
                byte* p1;
                int stride = dstdata.Stride;
                for (int i = 0; i < h; i++)
                {
                    for (int j = 0; j < w; j++)
                    {
                        p1 = p0 + 3 * j + i * stride;
                        H = sexiang(Convert.ToDouble(a[i, j]));
                        hsvTorgb(H, 1, 1, out r, out g, out b);
                        p1[2] = (byte)r;
                        p1[1] = (byte)g;
                        p1[0] = (byte)b;

                    }
                }
            }
            dst.UnlockBits(dstdata);

            return dst;
        }


        /// <summary>
        /// 功能函数——绘制灰度图，并修正深度数据，点到点投影到平面垂直距离
        /// </summary>
        /// <param name="b"></param>
        /// <param name="DepthData"></param>
        /// <param name="maxDis"></param>
        /// <returns></returns>
        public Bitmap drawImg(Bitmap b, int[,] DepthData, int maxDis)
        {
            Bitmap srcimg = (Bitmap)b.Clone();
            double hData = 0;
            int height = b.Height;
            int width = b.Width;

            //int x, y = 0;
            //double radius2 = 0;

            BitmapData srcdata = srcimg.LockBits(new Rectangle(0, 0, width, height), System.Drawing.Imaging.ImageLockMode.ReadWrite,
                               System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            unsafe
            {
                int stride = srcdata.Stride;

                // byte* p1 = (byte*)srcdata.Scan0 + stride * 6 + 12;//从第6行的第5个数据开始画，上下6行和左右4列数据无效
                byte* p1 = (byte*)srcdata.Scan0;
                int offset = stride - 3 * width;
                for (int i = 0; i < height; i++)
                {
                    for (int j = 0; j < width; j++)
                    {

                        // double f = 41;
                        double r = 0;

                        double fx = 197.24, fy = 197; //（DO12相机内参）
                        double cx = 168, cy = 123;
                        double thanTheta;
                        double cosTheta;//costheta余弦角度的具体数值
                        //double dis = 0;//定义实际的像素到光心的垂直距离
                        //dis = Math.Sqrt((j - 168) * (j - 168) + (i - 113) * (i - 113));
                        thanTheta = Math.Sqrt(((j - cx) * (j - cx) / fx / fx) + (i - cy) * (i - cy) / fy / fy);
                        //theta = Math.Cos(Math.Atan(f / (dis+0.01)));
                        // cosTheta = Math.Cos(Math.Atan(f / (dis + 0.01)));
                        cosTheta = Math.Cos(Math.Atan(thanTheta));

                        r = DepthData[i, j] * cosTheta;

                        hData = r * 255 / maxDis;
                        r = hData;

                        p1[0] = p1[1] = p1[2] = (byte)r;

                        p1 = p1 + 3;
                    }

                    p1 = p1 + offset;

                }
            }
            srcimg.UnlockBits(srcdata);
            return srcimg;
        }

       /// <summary>
       /// 功能函数——深度值转换
       /// </summary>
       /// <param name="DepthData"></param>
       /// <returns></returns>
        public int[,] DepthTrans(int[,] DepthData)
        {
            int height = DepthData.GetLength(0);
            int width = DepthData.GetLength(1);
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    double r = 0;

                    double fx = 197.24, fy = 197; //（DO12相机内参）
                    double cx = 168, cy = 123;
                    double tanTheta;
                    double cosTheta;//costheta余弦角度的具体数值

                    //double dis = 0;//定义实际的像素到光心的垂直距离
                    //dis = Math.Sqrt((j - 168) * (j - 168) + (i - 113) * (i - 113));
                    tanTheta = Math.Sqrt(((j - cx) * (j - cx) / fx / fx) + (i - cy) * (i - cy) / fy / fy);
                    //theta = Math.Cos(Math.Atan(f / (dis+0.01)));
                    //cosTheta = Math.Cos(Math.Atan(f / (dis + 0.01)));
                    cosTheta = Math.Cos(Math.Atan(tanTheta));

                    r = DepthData[i, j] * cosTheta;

                    DepthData[i, j] = (int)r;
                }

            }
            return DepthData;
        }

        /// <summary>
        /// 触发事件
        /// 获取目标图像的信息，深度数据进行处理，并显示
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button3srcImg_Click(object sender, EventArgs e)
        {
            int maxDistance = 0;
            OpenFileDialog ofg = new OpenFileDialog();
            //ofg.InitialDirectory = @"D:\win\2月23日\1"; 
            ofg.InitialDirectory = @"D:\win\new_client191\4.25\1.51";
            ofg.Filter = @"all files(*.*)|*.*";
            ofg.Title = "读入图像";
            if (ofg.ShowDialog() == DialogResult.OK)
            {
                string pic_name = ofg.FileName;

                //获取文件的行数
                string[] strRows = System.IO.File.ReadAllLines(pic_name);
                int j = 0;
                int[] opt;
                foreach (string str in strRows)
                {
                    //.csv文件转数组
                    string[] tmp = str.Split(new string[] { "," }, StringSplitOptions.None);
                    for (int i = 0; i < tmp.Length; i++)
                    {
                        //每行有328个数据
                        string[] output = new string[328];
                        output[i] = tmp[i].Replace("NaN", "0");
                        opt = Array.ConvertAll<string, int>(output, value => Convert.ToInt32(value));
                        if (opt[i] < 30000)
                        {
                            //深度数据转换对应的深度值（0-1250cm）
                            srcImgDepth[j, i] = opt[i] * 1250 / 30000;
                            srcImgData[j, i] = opt[i];
                        }
                        else
                        {
                            srcImgDepth[j, i] = 0;
                            srcImgData[j, i] = 0;
                        }

                        if (srcImgDepth[j, i] > maxDistance)
                        {
                            maxDistance = srcImgDepth[j, i];
                        }

                    }
                    j = j + 1;

                }
                Bitmap b = (Bitmap)pictureBox1.Image.Clone();
                Bitmap b1 = (Bitmap)pictureBox1.Image.Clone();
                Bitmap b2 = (Bitmap)pictureBox1.Image.Clone();

                //深度值转换
                srcImgDepth = DepthTrans(srcImgDepth);

                //灰度图像
                //b1 = drawImg(b, srcImgDepth, maxDistance);

                //RGB图像
                b1 = drawpic(srcImgData);

                // b1.RotateFlip(RotateFlipType.RotateNoneFlipY);
                pictureBox1.Image = b1;

                label5.Text = "目标图像"; 
            }
            else
            {
                MessageBox.Show("未加载图像！", "提示");
            }
        }

        /// <summary>
        /// 触发事件
        /// 图像背景差分
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4differImg_Click(object sender, EventArgs e)
        {
            if (pictureBox2.Image == null)
            {
                MessageBox.Show("错误提示：未加载图像！", "Error");
            }
            else
            {
                //实例化Bitmap，并初始化其尺寸
                Bitmap odb = (Bitmap)pictureBox2.Image.Clone();
                Bitmap image = new Bitmap(odb);
                int width = image.Width;
                int height = image.Height;

                //背景差分阈值t1、t2（可由界面数字选择控件调整）
                //默认为t1=200;t2=20
                int t1 = (int)numericUpDown1.Value;
                int t2 = (int)numericUpDown2.Value;

                unsafe
                {
                    BitmapData sData = image.LockBits(new Rectangle(0, 0, width, height),
                        ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);
                    int stride = sData.Stride;
                    int offset = stride - 3 * width;
                    int DifferenceData = 0;
                    byte* p = (byte*)sData.Scan0;
                    //double* p = (double*)sData.Scan0 + stride * 6 + 12;
                    for (int i = 0; i < height; i++)
                    {
                        for (int j = 0; j < width; j++)
                        {
                            DifferenceData = (int)Math.Abs(backImgDepth[i, j] - srcImgDepth[i, j]);
                            differImgData[j, i] = DifferenceData;

                            if (DifferenceData > t1 || DifferenceData < t2)
                            {
                                DifferenceData = 0;
                            }
                            // 3个颜色通道取值一致，以灰度图显示
                            p[0] = p[1] = p[2] = (byte)(DifferenceData);

                            p = p + 3;
                        }
                        p = p + offset;

                    }
                    image.UnlockBits(sData);
                    // image.RotateFlip(RotateFlipType.RotateNoneFlipY);
                    pictureBox3.Image = image;
                }
                label6.Text = "背景差分图像";
            }
            
        }

        /// <summary>
        /// 触发事件
        /// 图像中值滤波
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button5mediumFliter_Click(object sender, EventArgs e)
        {
            if (pictureBox3.Image == null)
            {
                MessageBox.Show("错误提示：未加载图像！", "Error");
            }
            else
            {
                Bitmap odb1 = (Bitmap)pictureBox3.Image.Clone();
                Bitmap image1 = new Bitmap((Bitmap)odb1.Clone()); //原始图像
                Bitmap image2 = new Bitmap((Bitmap)odb1.Clone()); //处理图像
                int width = image1.Width;
                int height = image1.Height;
                unsafe
                {
                    BitmapData sData = image1.LockBits(new Rectangle(0, 0, width, height),
                    ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);
                    BitmapData dData = image2.LockBits(new Rectangle(0, 0, width, height),
                    ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);

                    int stride = sData.Stride;
                    //byte* p = (byte*)sData.Scan0 + 12 + stride * 6;
                    //byte* pp = (byte*)dData.Scan0 + 12 + stride * 6;
                    byte* p = (byte*)sData.Scan0 + 3 + stride; //选模板
                    byte* pp = (byte*)dData.Scan0 + 3 + stride; //画新图

                    byte[] sequence = new byte[9];
                    int count;
                    int offset = stride - 3 * width;

                    for (int i = 1; i < height - 1; i++)
                    {
                        for (int j = 1; j < width - 1; j++)
                        {
                            //中值滤波3*3滤波模板
                            sequence[0] = (p - 3 - stride)[0];
                            sequence[1] = (p - stride)[0];
                            sequence[2] = (p + 3 - stride)[0];
                            sequence[3] = (p - 3)[0];
                            sequence[4] = (p + 3)[0];
                            sequence[5] = (p - 3 + stride)[0];
                            sequence[6] = (p + stride)[0];
                            sequence[7] = (p + 3 + stride)[0];
                            sequence[8] = p[0];

                            //由大到小排序
                            byte t = 0;
                            for (int k = 0; k < 8; k++)
                            {
                                for (int a = 1; a < 9; a++)
                                {
                                    if (sequence[a - 1] > sequence[a])
                                    {
                                        t = sequence[a - 1];
                                        sequence[a - 1] = sequence[a];
                                        sequence[a] = t;
                                    }
                                }
                            }

                            //选取中值
                            count = sequence[4];

                            //只选取中间区域进行中值滤波，四周区域直接滤除
                            if (j > 50 && j < 285 && i > 40 && i < 220)
                            {
                                pp[0] = pp[1] = pp[2] = (byte)(count);
                            }
                            else
                            {
                                pp[0] = pp[1] = pp[2] = 0;
                                (pp - 3)[0] = (pp - 3)[1] = (pp - 3)[2] = 0;
                                (pp + 3)[0] = (pp + 3)[1] = (pp + 3)[2] = 0;
                                (pp - stride - 3)[0] = (pp - stride - 3)[1] = (pp - stride - 3)[2] = 0;
                                (pp + stride + 3)[0] = (pp + stride + 3)[1] = (pp + stride + 3)[2] = 0;
                            }
                            p = p + 3;
                            pp = pp + 3;
                        }
                        //p = p + offset + 24;
                        //pp = pp + offset + 24;
                        p = p + offset + 6;
                        pp = pp + offset + 6;
                    }
                    image1.UnlockBits(sData);
                    image2.UnlockBits(dData);
                }
                pictureBox3.Image = image2;
            }

        }

        /// <summary>
        /// 触发事件
        /// 图像二值化处理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button7_Click(object sender, EventArgs e)
        {
            if(pictureBox3.Image == null)
            {
                MessageBox.Show("错误提示：未加载图像！", "Error");
            }
            else
            {
                Bitmap odb1 = (Bitmap)pictureBox3.Image.Clone();
                Bitmap image1 = new Bitmap(odb1);

                int width = image1.Width;
                int height = image1.Height;
                int t = (int)numericUpDown4.Value;//二值化阈值
                unsafe
                {
                    BitmapData sData = image1.LockBits(new Rectangle(0, 0, width, height),
                        ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb); //3通道，各8位
                    int gray;
                    int stride = sData.Stride;
                    // 左右4、上下6
                    byte* p1 = (byte*)sData.Scan0 + 12 + stride * 6;
                    int offset = stride - 3 * width;
                    for (int i = 6; i < height - 6; i++)
                    {
                        for (int j = 4; j < width - 4; j++)
                        {
                            //gray = Math.Abs(p2[0] - p1[0]);  111-214  5-89
                            gray = Math.Abs(p1[0]);
                            if (gray > t)
                            {
                                p1[0] = p1[1] = p1[2] = 255;
                            }
                            else

                                p1[0] = p1[1] = p1[2] = 0;

                            p1 = p1 + 3;
                        }
                        p1 = p1 + offset + 24;
                    }
                    image1.UnlockBits(sData);
                    image1 = MoveSmallAreas(image1);
                }
                pictureBox4.Image = image1;

                label7.Text = "二值化图像";
            }
         
        }

        /// <summary>
        /// 功能函数：剔除小面积区域
        /// 部分噪声不易被滤波，故采用此方式进行剔除
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public Bitmap MoveSmallAreas(Bitmap image)
        {
            int width = image.Width;
            int height = image.Height;
            Image<Gray, byte> src = new Image<Gray, byte>(image);
            unsafe
            {
                BitmapData sData = image.LockBits(new Rectangle(0, 0, width, height),
                   ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);
                byte* p1 = (byte*)sData.Scan0;
                int stride = sData.Stride;

                using (VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint())
                {
                    //扫描连通域
                    CvInvoke.FindContours(src, contours, null, RetrType.External,
                                            ChainApproxMethod.ChainApproxSimple);

                    int count = contours.Size;//轮廓个数，也是连通域的个数
                    //List<double> areas = new List<double>(); //存放连通域面积
                    Rectangle rect;

                   // Console.WriteLine("contours.Size is" + count);
                    for (int i = 0; i < count; i++)
                    {
                        VectorOfPoint temp = contours[i];         //存放每个连通域的坐标参数
                        double area = CvInvoke.ContourArea(temp); //各个连通域面积

                       // Console.WriteLine("A is" + area);
                        //areas.Add(area);

                        rect = CvInvoke.BoundingRectangle(contours[i]);
                        //int x1 = rect.X;
                        //int y1 = rect.Y;
                        //p1 = p1 + x1 * 3 + y1 * stride;

                        //Console.WriteLine("x is" + x1);
                        //Console.WriteLine("y is" + y1);

                        if (area < 200) //剔除小面积区域
                        {
                            for (int y = rect.Y; y < rect.Y + rect.Height; y++)
                            {
                                for (int x = rect.X; x < rect.X + rect.Width; x++)
                                {
                                    p1 = (byte*)sData.Scan0 + x * 3 + y * stride;
                                    p1[0] = p1[1] = p1[2] = 0;
                                }

                            }
                        }
                    }
                }
                image.UnlockBits(sData);
            }

            return image;

        }

        /// <summary>
        /// 触发事件
        /// 图像模板滤波处理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button6binary_Click(object sender, EventArgs e)
        {
            if (pictureBox3.Image == null)
            {
                MessageBox.Show("错误提示：未加载图像！", "Error");
            }
            else
            {
                Bitmap a = new Bitmap(pictureBox3.Image);
                pictureBox3.Image = FilterImage(a); //模板滤波
            }

        }

        /// <summary>
        /// 模板滤波
        /// </summary>
        /// <param name="a">输入图像</param>
        /// <returns>b 输出图像</returns>
        public Bitmap FilterImage(Bitmap a)
        {
            Bitmap b = (Bitmap)a.Clone();
            int width = b.Width;
            int height = b.Height;
            int t = (int)numericUpDown3.Value;
            unsafe
            {
                BitmapData sData = b.LockBits(new Rectangle(0, 0, width, height),
                    ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);

                int PiexlSum;
                int stride = sData.Stride;
                int offset = stride - 3 * width;
                byte* p = (byte*)sData.Scan0 + stride * 9 + 21;//6+3  4+3


                for (int j = 9; j < height - 9; j++)
                {
                    for (int i = 7; i < width - 7; i++)
                    {

                        byte* p1 = p - stride * 2;
                        byte* p2 = p - stride;
                        byte* p4 = p + stride;
                        byte* p5 = p + stride * 2;
                        PiexlSum = ((4 * (p1 - 6)[0] + 4 * (p1 - 3)[0] + 6 * p1[0] + 5 * (p1 + 3)[0] + 7 * (p1 + 6)[0] +
                                     4 * (p2 - 6)[0] + 5 * (p2 - 3)[0] + 5 * p2[0] + 5 * (p2 + 3)[0] + 5 * (p2 + 6)[0] +
                                     5 * (p - 6)[0] + 6 * (p - 3)[0] + 7 * p[0] + 4 * (p + 3)[0] + 5 * (p + 6)[0] +
                                     6 * (p4 - 6)[0] + 1 * (p4 - 3)[0] + 5 * p4[0] + 4 * (p4 + 3)[0] + 5 * (p4 + 6)[0] +
                                     5 * (p5 - 6)[0] + 7 * (p5 - 3)[0] + 6 * p5[0] + 5 * (p5 + 3)[0] + 5 * (p5 + 6)[0]) / 25);


                        if (PiexlSum <= t)
                        {
                            p[0] = p[1] = p[2] = 0;
                        }

                        p = p + 3;

                    }
                    p = p + offset + 42;

                }
                b.UnlockBits(sData);
                //b.UnlockBits(dData);
            }
            return b;
        }

        /// <summary>
        /// 触发事件
        /// 画出外接矩形
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button8_Click(object sender, EventArgs e)
        {
            if (pictureBox4.Image == null)
            {
                MessageBox.Show("错误提示：暂无数据！", "Error");
            }
            else
            {
                Bitmap b = (Bitmap)pictureBox4.Image.Clone();

                Image<Gray, byte> src = new Image<Gray, byte>(b); //灰度图
                Image<Bgr, byte> DrawI = src.Convert<Bgr, byte>(); //画框 RGB图

                MinAreaBoundingBox(src, DrawI); //画框并计算

                pictureBox4.Image = DrawI.Bitmap;
            }
        }

        /// <summary>
        /// 功能函数——画出最小外接矩形，并计算两边边长
        /// </summary>
        /// <param name="src"></param>
        /// <param name="draw"></param>
        public void MinAreaBoundingBox(Image<Gray, byte> src, Image<Bgr, byte> draw)
        {
            using (VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint())
            {
                //找到轮廓
                CvInvoke.FindContours(src, contours, null, RetrType.External,
                                        ChainApproxMethod.ChainApproxSimple);

                int count = contours.Size;//轮廓个数，也是连通域的个数
                //while (count != null) 
                // {
                //     RotatedRect rect = CvInvoke.MinAreaRect(contours);
                //    if(rect.MinAreaRect<10)
                // }
                Console.WriteLine("contours.Size is" + count);
                for (int i = 0; i < count; i++)
                {
                    using (VectorOfPoint contour = contours[i])
                    {
                        //对每一个连通域做处理
                        int size = contour.Size;
                        Console.WriteLine(size);

                        //实例化旋转矩形类，RotatedRect，包含质心、边长、旋转角度三个属性
                        RotatedRect BoundingBox = CvInvoke.MinAreaRect(contour);
                        PointF pointC;
                        int k;
                        if (BoundingBox.Size.Height > 10 && BoundingBox.Size.Width > 10)//连通域面积太小不画矩形
                        {
                            pointC = BoundingBox.Center;
                            Console.WriteLine("CenterPoint is" + pointC);
                            listBox1.Items.Add("中心:" + pointC);
                            
                            //获取连通域标记符号，从而可以调用对应的单位像素对应实际距离的映射比p240
                            k = newsignall[(int)pointC.X, (int)pointC.Y]; 
                            //Console.WriteLine("k = " + k);
                            //Console.WriteLine("pk = " + p240s[k - 1]);

                            actWidth = BoundingBox.Size.Width * p240s[k-1];
                            textBox12.Text = actWidth.ToString("f2") + "cm";//长度
                            listBox1.Items.Add("长度:" + actWidth.ToString("f2") + "cm");

                            aWidth = (float)actWidth;
                            actHeight = BoundingBox.Size.Height * p240s[k - 1];
                            textBox13.Text = actHeight.ToString("f2") + "cm";//宽度
                            listBox1.Items.Add("宽度:" + actHeight.ToString("f2") + "cm");

                            aHeight = (float)actHeight;


                            //2个盒子的，大盒子长18cm宽16cm高8.5cm  小盒子长12cm宽11.5cm高9cm
                            Console.WriteLine("width is {0},height is {1}", actWidth, actHeight);

                            PointF[] fourPoint = new PointF[4];
                            fourPoint = BoundingBox.GetVertices();

                            foreach (PointF p in fourPoint)
                            {
                                Console.WriteLine("4 coners of this counter are " + p);
                            }

                            CvInvoke.Polylines(draw, Array.ConvertAll(BoundingBox.GetVertices(), Point.Round),
                                               true, new Bgr(Color.DeepPink).MCvScalar, 1);
                        }
                    }

                }


            }

        }


        /// <summary>
        /// 触发事件
        /// 盒子高度计算——连通域内深度值求平均
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button9_Click(object sender, EventArgs e)
        {
            if (pictureBox4.Image == null)
            {
                MessageBox.Show("错误提示：暂无数据！", "Error");
            }
            else
            {
                Bitmap areaimage = new Bitmap(pictureBox4.Image);//二值图，只剩目标区域连通域

                Bitmap b1 = new Bitmap(pictureBox4.Image);
                Bitmap b2 = new Bitmap(pictureBox4.Image);
                int w = areaimage.Width;
                int h = areaimage.Height;
                //图像转数组
                int[,] b = Image2D(areaimage, w, h);

                //得到图像的标记图（与图像同尺寸的二维数组），每个坐标都有对应的标记
                ushort[,] signall = ImageSign(b, w, h);
                newsignall = ImageSign(b, w, h);

                //测试。。查看标记的样子
                //showpic(newsignall);

                //连通域的个数
                connectareanumber = ImageAreas(newsignall, out max);

                int[] actulength = new int[connectareanumber];
                
                //计算高度
                actulength = Actuallength(signall, w, h, connectareanumber, differImgData);//均为原始数据
                connectAreaFlag = true;
            }
        }

        //private void showpic(ushort[,] sign)
        //{
        //    int w = sign.GetLength(0);
        //    int h = sign.GetLength(1);
        //    Bitmap pic = (Bitmap)pictureBox1.Image.Clone();

        //    for(int i = 0; i < w; i++)
        //    {
        //        for(int j = 0; j < h; j++)
        //        {
        //            Color c = Color.FromArgb((int)sign[i, j] * 50, (int)sign[i, j] * 50, (int)sign[i, j] * 50);
        //            pic.SetPixel(i, j, c);
        //        }
        //    }

        //    pictureBox5.Image = pic;


        //}


        //  public Bitmap PangzhangFushi()
        //{
        //    {
        //        string strFileName = string.Empty;
        //        OpenFileDialog ofd = new OpenFileDialog();
        //        if (ofd.ShowDialog() == DialogResult.OK)
        //        {
        //            IntPtr img = CvInvoke.cvLoadImage(ofd.FileName, Emgu.CV.CvEnum.LOAD_IMAGE_TYPE.CV_LOAD_IMAGE_GRAYSCALE);
        //            Image<Bgr, Byte> simage =new Image<Bgr, byte>(CvInvoke.cvGetSize(img));  
        //            Image<Gray, Byte> dst = new Image<Gray, Byte>(CvInvoke.cvGetSize(img));
        //            Image<Gray, Byte> depth_Threshold1 = new Image<Gray, Byte>(CvInvoke.cvGetSize(img));
        //            Image<Gray, Byte> depth_Threshold2 = new Image<Gray, Byte>(CvInvoke.cvGetSize(img));
        //            Image<Gray, Byte> depth_Morphic1 = new Image<Gray, Byte>(CvInvoke.cvGetSize(img));
        //            Image<Gray, Byte> depth_Contour1 = new Image<Gray, Byte>(CvInvoke.cvGetSize(img));

        //            CvInvoke.cvCopy(img, dst, IntPtr.Zero);
        //            pictureBox1.Image = dst.ToBitmap();

        //            //阈值操作后的深度图像
        //            IntPtr depth_Threshold = CvInvoke.cvCreateImage(CvInvoke.cvGetSize(img), Emgu.CV.CvEnum.IplDepth.IplDepth_8U, 1);
        //            //形态学操作后的深度图像
        //            IntPtr depth_Morphic = CvInvoke.cvCreateImage(CvInvoke.cvGetSize(img), Emgu.CV.CvEnum.IplDepth.IplDepth_8U, 1);
        //            //进行边界提取后的深度图像
        //            IntPtr depth_Contour = CvInvoke.cvCreateImage(CvInvoke.cvGetSize(img), Emgu.CV.CvEnum.IplDepth.IplDepth_8U, 1);

        //            //单一阈值分割
        //            CvInvoke.Threshold(img, depth_Threshold, 60, 255,ThresholdType.ToZero);
        //            CvInvoke.cvCopy(depth_Threshold, depth_Threshold1, IntPtr.Zero);
        //            pictureBox2.Image = depth_Threshold1.ToBitmap();
        //            //开运算
        //            CvInvoke.Erode(depth_Threshold, depth_Threshold, IntPtr.Zero, 1);
        //            //CvInvoke.cvErode(depth_Threshold, depth_Threshold, IntPtr.Zero, 1);
        //            CvInvoke.cvCopy(depth_Threshold, depth_Threshold2, IntPtr.Zero);
        //            CvInvoke.Dilate(depth_Threshold, depth_Morphic, IntPtr.Zero,(-1,-1),,);
        //            CvInvoke.cvCopy(depth_Morphic, depth_Morphic1, IntPtr.Zero);

        //            pictureBox3.Image = depth_Morphic1.ToBitmap();

        //            //   imageBox5.Image = depth_Contour1;
        //            using (MemStorage stor = new MemStorage())
        //            {
        //                Contour<Point> contours = depth_Morphic1.FindContours(
        //               Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_SIMPLE,
        //               Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_CCOMP,
        //               stor);
        //                for (; contours != null; contours = contours.HNext)
        //                {

        //                    Rectangle box = contours.BoundingRectangle;
        //                    Image<Bgr, Byte> test = simage.CopyBlank();
        //                    test.SetValue(255.0);
        //                    double whRatio = (double)box.Width / box.Height;
        //                    int area = (int)box.Width * box.Height;
        //                    if (area > 10)
        //                    {

        //                            test.Draw(box, new Bgr(Color.Red), 2);
        //                            Image<Gray, byte> img2 = simage.Canny(50, 100);
        //                            simage.Draw(box, new Bgr(Color.Red), 2);//CvInvoke.cvNamedWindow("dst");
        //                            //CvInvoke.cvShowImage("dst", dst);
        //                            pictureBox4.Image = simage.ToBitmap();

        //                    }
        //                }

        //            }
        //        }

        //    }


        //}


        //public Bitmap deletMiniArea(ushort[,] signall,int connectNo, int[,] differDepthData) 
        //{
        //    Bitmap src=(Bitmap) pictureBox3.Image.Clone();
        //    int h=src.Height;
        //    int w=src.Width;
        //    int[] depthNo = new int[connectNo];//统计每个连通域内的像素的总个数           
        //    int[] differDepthDataSum = new int[connectNo];//统计每个连通域内的像素的深度值的和    
        //    int[] avgDifferDepthData = new int[connectNo];//每个连通域的平均深度

        //     for (int k = 1; k < connectNo + 1; k++)
        //    {
        //        for (int j = 0; j < h; j++)
        //        {
        //            for (int i = 0; i < w; i++)
        //            {
        //                if (signall[i, j] == k)
        //                {
        //                    depthNo[k - 1]++;//统计连通域内的像素个数                                                    

        //                }
        //            }
        //        }
        //}



        /// <summary>
        /// 功能函数
        /// 图像数据Bitmap类转二维数组
        /// </summary>
        /// <param name="b"></param>
        /// <param name="w"></param>
        /// <param name="h"></param>
        /// <returns></returns>
        public int[,] Image2D(Bitmap b, int w, int h)
        {
            // 申请一个二维数组
            int[,] GrayArray = new int[w, h];//数组大小是[320，240]320行，240列

            BitmapData data = b.LockBits(new Rectangle(0, 0, w, h),
              ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);

            unsafe
            {
                byte* p = (byte*)data.Scan0; //p[0][1][2]三通道
                int offset = data.Stride - w * 3;

                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        GrayArray[x, y] = p[0];//数组坐标是以320作为x轴坐标，240作为Y坐标
                        p += 3;
                    } //  x
                    p += offset;
                } // y
            }
            b.UnlockBits(data);
            return GrayArray;
        }

        /// <summary>
        /// 功能函数：标记连通域
        /// 标记号，最多可以标记 65536 个不同的连通区域
        /// 注意标记号从 1 开始依次递增，标记号 0 代表背景
        /// </summary>
        /// <param name="b"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <returns></returns>
        public ushort[,] ImageSign(int[,] b, int width, int height)
        {
            // int width = b.GetLength(0);//320
            //int height = b.GetLength(1);//240
          
            ushort signNo = 1;

            // 用堆栈记录所有空标记
            Stack Seat = new Stack();

            // 二值图像连通区域标记，存储的是区域标记，而非图像数据
            ushort[,] Sign = new ushort[width, height];


            // 初始化最顶行标记
            for (int x = 0; x < width; x++)//320列
            {
                if (b[x, 0] != 255) continue;

                // 处理所有连续的白点
                while (x < width && b[x, 0] == 255)
                {
                    Sign[x, 0] = signNo;//标记从1开始
                    x++;
                }
                signNo++;
            }

            // 处理最左列及最右列标记
            for (int y = 1; y < height; y++)
            {
                // 最左列
                if (b[0, y] == 255)
                {
                    if (b[0, y - 1] == 255)//如果最左列和正上方的像素均为白色，则做相同标记，否则新标记
                        Sign[0, y] = Sign[0, y - 1];
                    else
                        Sign[0, y] = signNo++;
                }

                // 最右列
                if (b[width - 1, y] == 255)
                {
                    if (b[width - 1, y - 1] == 255)//如果最右列和正上方的像素均为白色，则做相同标记，否则新标记
                        Sign[width - 1, y] = Sign[width - 1, y - 1];
                    else
                        Sign[width - 1, y] = signNo++;
                }
            }

            // 上面已经处理了图像最顶行、最左列、最右列，
            // 故下面只处理排除这三行后的主图像区
            int topRect = 1;
            int bottomRect = height;
            int leftRect = 1;
            int rightRect = width - 1;

            // 从左到右开始标记
            for (int y = topRect; y < bottomRect; y++)
            {
                for (int x = leftRect; x < rightRect; x++)
                {
                    // 如果当前点不为白点，则跳过不处理----只对白色点继续下面的操作
                    if (b[x, y] != 255) continue;

                    // 右上
                    if (b[x + 1, y - 1] == 255)//当前点与右上点均为白点时做相同标记
                    {
                        // 将当前点置为与右上点相同的标记
                        ushort sign = Sign[x, y] = Sign[x + 1, y - 1];

                        // 当左前点为白点，且左前点的标记与右上点的标记不同时
                        if (b[x - 1, y] == 255 && Sign[x - 1, y] != sign)
                        {
                            // 进栈：记录左前点标记，因为该标记将被替换掉
                            Seat.Push(Sign[x - 1, y]);

                            // 用右上点的标记号替换掉所有与左前点标记号相同的点
                            ReplaceSign(ref Sign, Sign[x - 1, y], sign);
                        }

                        // 当左上点为白点，且左上点的标记与右上点的标记不同时
                        else if (b[x - 1, y - 1] == 255 && Sign[x - 1, y - 1] != sign)
                        {
                            // 进栈：记录左上点标记，因为该标记将被替换掉
                            Seat.Push(Sign[x - 1, y - 1]);

                            // 用右上点的标记号替换掉所有与左上点标记号相同的点
                            ReplaceSign(ref Sign, Sign[x - 1, y - 1], sign);
                        }
                    } // 右上完

                    // 正上
                    else if (b[x, y - 1] == 255)
                    {
                        // 将当前点置为与正上点相同的标记
                        Sign[x, y] = Sign[x, y - 1];
                    }

                    // 左上
                    else if (b[x - 1, y - 1] == 255)
                    {
                        // 将当前点置为与左上点相同的标记
                        Sign[x, y] = Sign[x - 1, y - 1];
                    }

                    // 左前
                    else if (b[x - 1, y] == 255)
                    {
                        // 将当前点置为与左前点相同的标记
                        Sign[x, y] = Sign[x - 1, y];
                    }

                    // 右上、正上、左上及左前四点均不为白点，即表示新区域开始
                    // * * *
                    // * P
                    else
                    {
                        // 避免区域数超过 0xFFFF
                        if (signNo >= 0xFFFF)
                            return Sign;

                        // 如果堆栈里无空标记，
                        // 则使用新标记，否则使用该空标记
                        if (Seat.Count == 0)
                            Sign[x, y] = signNo++;
                        else
                            Sign[x, y] = (ushort)Seat.Pop(); // 出栈
                    }

                } // x
            } // y

            // 堆栈里存在空标记，则使用完没有空标记
            while (Seat.Count > 0)
            {
                ReplaceSign(ref Sign, (ushort)(--signNo), (ushort)Seat.Pop());
            }
            return Sign;
        } // 标记结束


        /// <summary>
        /// 用新的标记号替换掉标记数组中旧的标记号
        /// </summary>
        /// <param name="Sign">二值图像标记数组</param>
        /// <param name="srcSign">原始标记号</param>
        /// <param name="dstSign">目标标记号</param>
        private void ReplaceSign(ref ushort[,] Sign, ushort srcSign, ushort dstSign)
        {
            int width = Sign.GetLength(0);
            int height = Sign.GetLength(1);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    if (Sign[x, y] == srcSign)
                        Sign[x, y] = dstSign;
                } // x
            } // y
        } // end of ReplaceSign

        
        /// <summary>
        /// 功能函数：计算连通域个数
        /// </summary>
        /// <param name="Sign"></param>
        /// <param name="max"></param>
        /// <returns></returns>
        public int ImageAreas(ushort[,] Sign, out int max)
        {
            int width = Sign.GetLength(0);
            int height = Sign.GetLength(1);
            Bitmap dst = new Bitmap(width, height);

            max = 0;
            // 找出最大标记号，即找出区域数，即连通域的个数

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    if (Sign[x, y] > max)
                        max = Sign[x, y];
                } // x
            } // y
            return max;
            // // 面积统计数组
            // int[] Area = new int[max + 1];
            //// int Max = 0;
            //// int[, ,] sign = new int[width, height, 2];
            // // 计算区域面积
            // for (int y = 0; y < height; y++)
            // {
            //     for (int x = 0; x < width; x++)
            //     {
            //         if (Sign[x, y] == 0) continue;//标记为0代表此时的是背景，则跳过不处理
            //         Area[Sign[x, y]]++;//标记不为0时，Area[]++

            //     } // x
            // }// y  
            // for (int y = 0; y < height; y++)
            // {
            //     for (int x = 0; x < width; x++)
            //     {
            //         if (Area[Sign[x, y]] < 100)
            //         {
            //             Sign[x, y] = 0; ;

            //         }

            //     } // x
            // } // 



        }


        /// <summary>
        /// 功能函数：计算各连通域内的平均深度值
        /// </summary>
        /// <param name="signall">所有已标记的连通域</param>
        /// <param name="w">图像尺寸，宽度</param>
        /// <param name="h">图像尺寸，高度</param>
        /// <param name="connectNo">连通域数量</param>
        /// <param name="differDepthData">深度信息</param>
        /// <returns></returns>
        public int[] Actuallength(ushort[,] signall, int w, int h, int connectNo, int[,] differDepthData)
        {
            //注：标记0为背景，1开始为连通域
            int[] depthNo = new int[connectNo];//统计每个连通域内的像素的总个数           
            int[] differDepthDataSum = new int[connectNo];//统计每个连通域内的像素的深度值的和    
            int[] avgDifferDepthData = new int[connectNo];//每个连通域的平均深度

            //对每个连通域进行操作
            for (int k = 1; k < connectNo + 1; k++)
            {
                //统计并计算当前连通域的像素个数及其深度均值
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        if (signall[i, j] == k)
                        {
                            depthNo[k - 1]++;//统计连通域内的像素个数

                            differDepthDataSum[k - 1] = differDepthDataSum[k - 1] + differDepthData[i, j];

                        }
                    }
                }

                avgDifferDepthData[k - 1] = (int)(differDepthDataSum[k - 1] / (depthNo[k - 1]));//盒子的高则为深度差分后box3对应的深度差分数据所有和的平均值

                Console.Write("   高度:");
                Console.Write(avgDifferDepthData[k - 1]);
                aDepth = avgDifferDepthData[k - 1];
                
                //目前相机架起高度为240cm
                p240 = (CameraDepth - aDepth) / 192;

                p240s[k - 1] = p240;

                Console.Write("   p240:" + p240 + "    p240s:" + p240s[k - 1]);

                textBox14.Text = avgDifferDepthData[k - 1].ToString() + "cm";//显示高度

                listBox1.Items.Add("depth" + (k).ToString() + "=" + avgDifferDepthData[k - 1].ToString() + "cm");

            }
            // return avgDepth;
            return avgDifferDepthData;

        }



        /// <summary>
        /// 触发事件
        /// 将深度测量结果传递给sharpGL绘制矩形形状
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button13_Click(object sender, EventArgs e)
        {

            float a = aWidth;
            //float a = textBox12.Text;
            float b = aHeight;
            float c = aDepth;

            drawShape(a, b, c);
        }

        //private void openGLControl1_OpenGLDraw(object sender, SharpGL.RenderEventArgs args)
        //{
        //    float a = 2;
        //    float b = 3;
        //    float c = 4;
        //    drawShape(a, b, c);       
        //}


        /// <summary>
        /// 功能函数
        /// sharpGL绘制矩形形状
        /// </summary>
        /// <param name="alength"></param>
        /// <param name="awidth"></param>
        /// <param name="ahight"></param>
        public void drawShape(float alength, float awidth, float ahight)
        {
            _x = (float)(numericUpDown5.Value);
            _y = (float)(numericUpDown6.Value);
            _z = (float)(numericUpDown7.Value);

            double x, y, z;
            x = alength / 4;
            y = awidth / 4;
            z = ahight / 4;
            //lengthWidthHeight(actWidth,actHeight, avgDifferDepthData[0], out x, out y, out z);

            SharpGL.OpenGL gl = new OpenGL();       //创建OpenGL对象

            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);

            gl.LoadIdentity();            //清除深度缓存

            //重置模型观察矩阵，其实就是重置了三维坐标轴的位置，将其初始化为原点

            gl.Translate(0f, 0f, -60f);  //最后一个参数为距离屏幕距离


            //图像旋转

            gl.Rotate(_x, 1.0f, 0.0f, 0.0f); // 绕X轴旋转

            gl.Rotate(_y, 0.0f, 1.0f, 0.0f); // 绕Y轴旋转

            gl.Rotate(_z, 0.0f, 0.0f, 1.0f); // 绕Z轴旋转



            //gl.Begin(OpenGL.GL_LINE_LOOP);           // Start Drawing The Cube
            gl.Begin(OpenGL.GL_QUADS);
            //长方体6个面绘制（1） 底面
            // gl.Color(0.0f, 0.0f, 1.0f); 
            gl.Color(0.5f, 0.5f, 1.0f);

            gl.Vertex(0.0f, 0.0f, 0.0f);

            gl.Vertex(0.0f, y, 0.0f);

            gl.Vertex(x, y, 0.0f);

            gl.Vertex(x, 0.0f, 0.0f);

            gl.End();         // DoneDrawing The Q


            //（2）后面

            //gl.Begin(OpenGL.GL_LINE_LOOP);
            gl.Begin(OpenGL.GL_QUADS);

            gl.Color(1.0f, 0.0f, 0.0f);

            gl.Vertex(0.0f, 0.0f, 0.0f);

            gl.Vertex(0.0f, 0.0f, z);

            gl.Vertex(0.0f, y, z);

            gl.Vertex(0.0f, y, 0.0f);

            gl.End();


            //（3）右面

            //gl.Begin(OpenGL.GL_LINE_LOOP);

            gl.Begin(OpenGL.GL_QUADS);

            gl.Color(0.0f, 1.0f, 0.0f);

            gl.Vertex(0.0f, y, 0.0f);

            gl.Vertex(0.0f, y, z);

            gl.Vertex(x, y, z);

            gl.Vertex(x, y, 0.0f);

            gl.End();

            //（4）前面

            // gl.Begin(OpenGL.GL_LINE_LOOP);

            gl.Begin(OpenGL.GL_QUADS);

            gl.Color(0.0f, 0.0f, 1.0f);

            gl.Vertex(x, 0.0f, 0.0f);

            gl.Vertex(x, 0.0f, z);

            gl.Vertex(x, y, z);

            gl.Vertex(x, y, 0.0f);
            gl.End();

            //（5）左面

            // gl.Begin(OpenGL.GL_LINE_LOOP);

            gl.Begin(OpenGL.GL_QUADS);

            gl.Color(0.4f, 1.0f, 0.6f);

            gl.Vertex(0.0f, 0.0f, 0.0f);

            gl.Vertex(x, 0.0f, 0.0f);

            gl.Vertex(x, 0.0f, z);

            gl.Vertex(0.0f, 0.0f, z);

            gl.End();

            //（6）上面

            //gl.Begin(OpenGL.GL_LINE_LOOP);

            gl.Begin(OpenGL.GL_QUADS);

            gl.Color(3.0f, 1.0f, 0.6f);

            gl.Vertex(0.0f, 0.0f, z);

            gl.Vertex(0.0f, y, z);

            gl.Vertex(x, y, z);

            gl.Vertex(x, 0.0f, z);

            gl.End();
        }



        /// <summary>
        /// 触发事件
        /// 连接相机
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button10_Click(object sender, EventArgs e)
        {
            string ip = textBox3.Text.ToString();
            dfip = IPAddress.Parse(ip);               // 192.168.7.2
            dfport = Convert.ToInt32(textBox4.Text);  // 50660
            try
            {
                clintSocket.Connect(new IPEndPoint(dfip, dfport));
                textBox5.Text = "连接成功";
                textBox5.BackColor = Color.Green;
                textBox5.ForeColor = Color.White;
            }
            catch
            {
                textBox5.Text = "连接失败";
                textBox5.BackColor = Color.Red;
            }
            if(clintSocket.Connected)
            {
                iscollected = true;
                clintSocket.Shutdown(SocketShutdown.Both);
                clintSocket.Close();
            }
            else
            {
                MessageBox.Show("相机未连接！", "Error");
            }

            
        }

        /// <summary>
        /// 功能函数
        /// 向相机发送指令，得到深度图像
        /// </summary>
        /// <param name="dis"></param>
        /// <param name="amp"></param>
        private void getdistanceandamplitudesorted(out byte[] dis)
        {
            if(iscollected)
            {
                result = new byte[153600];  // 深度图像数据以字形式传输，320*240*2
                clintSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                clintSocket.Connect(dfip, dfport); //连接相机
                int receiveleg = 1;
                int crt = 0;
                // string sendmessage = "getDistanceAndAmplitudeSorted";
                string sendmessage = "getDistanceSorted";  //请求数据指令
                try
                {
                    clintSocket.Send(Encoding.ASCII.GetBytes(sendmessage));
                    do
                    {
                        receiveleg = clintSocket.Receive(result, crt, 153600 - crt, SocketFlags.None);
                        crt += receiveleg;
                    } while (receiveleg > 0); // 接收完整一帧图像数据时停止
                }
                catch
                {
                }
                clintSocket.Shutdown(SocketShutdown.Both);
                clintSocket.Close();
                dis = new byte[153600];

                for (int i = 0; i < 153600; i++)
                {
                    dis[i] = result[i];

                }

            }
            else
            {
                MessageBox.Show("相机未连接！", "Error");
                dis = null;
            }
           
        }

        /// <summary>
        /// 功能函数
        /// 将一维的数据转换二维数组
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public int[,] datatranshalf(byte[] source)
        {
            int lg = source.Length;
            int[,] r1 = new int[320, 240];
            int[,] r2 = new int[320, 240];
            int[,] pic = new int[320, 240];
            int line = 0, row = 0;
            for (int i = 0; i < lg; i = i + 2)
            {
                if (row < 320)
                {
                    r1[row, line] = source[i];
                    r2[row, line] = source[i + 1];
                    row++;
                }
                else
                {
                    row = 0;//大于320时候，换行
                    r1[row, line] = source[i];
                    r2[row, line] = source[i + 1];
                    row++;
                    line++;
                }
            }
            for (int i = 0; i < 240; i++)
            {
                for (int j = 0; j < 320; j++)
                {
                    pic[j, i] = r1[j, i] + r2[j, i] * 256;  //字转换
                }
            }
            return pic;
        }
        
        
        /// <summary>
        /// 功能函数
        /// 二维转一维[240,320]
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public int[,] datatranshalf1to2(ushort[] source)
        {
            int[,] pic = new int[240, 320];
            for (int i = 0; i < 240; i++)
            {
                for (int j = 0; j < 320; j++)
                {
                    pic[i, j] = source[j + 320 * i];
                }
            }
            return pic;
        }


        /// <summary>
        /// 功能函数
        /// 将颠倒的数据翻转 320*240
        /// </summary>
        /// <param name="souce"></param>
        /// <returns></returns>
        public int[,] displace(int[,] souce)
        {
            int w = souce.GetLength(0);  //320
            int h = souce.GetLength(1);  //240
            int[,] a = new int[h, w];  // [240,320]

            for (int i = 0; i < h; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    a[i, j] = souce[j, i];//i=240  j=320
                }
            }

            return a;
        }

        /// <summary>
        /// 功能函数
        /// 深度信息转rgb图像数据
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        public Bitmap drawquaterpic(int[,] source)  
        {
            int h = source.GetLength(0);
            int w = source.GetLength(1);
            int[,] a = new int[h, w];               //[240,320]--320*240

            for (int i = 0; i < h; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    a[i, j] = source[i, j];
                }
            }
            Bitmap dst = new Bitmap(w, h);
            
            //访问Bitmap类数据，指定格式为每像素 24 位；红色、绿色和蓝色分量各使用8位。
            BitmapData dstdata = dst.LockBits(new Rectangle(0, 0, w, h), 
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            double r, g, b, H;

            unsafe
            {
                byte* p0 = (byte*)dstdata.Scan0; //获取位图中第一个像素数据的地址
                byte* p1;
                int stride = dstdata.Stride; //扫描宽度
                for (int i = 0; i < h; i++)
                {
                    for (int j = 0; j < w; j++)
                    {
                        //3通道，故指针p每次要（3 * j）
                        p1 = p0 + 3 * j + i * stride;

                        //剔除无效数据
                        if (a[i, j] == 30000 || a[i, j] == 0)
                        {
                            p1[2] = p1[1] = p1[0] = 0;
                        }
                        else
                        {
                            H = sexiang(Convert.ToDouble(a[i, j]));   //获得H值
                            hsvTorgb(H, 1, 1, out r, out g, out b);   //已获得色相H（0-360），由HSV空间转向RGB，从纯彩色开始，令V=S=1
                            p1[2] = (byte)r;
                            p1[1] = (byte)g;
                            p1[0] = (byte)b;

                        }
                    }
                }
            }
            dst.UnlockBits(dstdata);

            return dst;
        }

        /// <summary>
        /// 功能函数
        /// 根据距离信息映射得到H值（色相）
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public double sexiang(double value)
        {
            double h = 0;
            double flag = 0;
            if (value > 0 && value < 2999)   // h  225-255
            {
                flag = value / 100;
                h = 225 + flag;//flag+225
            }
            else if (value < 20000)   // h 0-169
            {
                flag = value / 100;
                h = flag - 30;///flag-30
            }
            else if (value < 30000)    // h 170-224
            {
                flag = (value - 20000) / 185;
                h = flag + 170;///flag+170
            }
            h = h * 360 / 255;
            return h;
        }


        /// <summary>
        /// 功能函数
        /// 将HSV的参数转化为RGB参数输出
        /// </summary>
        /// <param name="h"></param>
        /// <param name="s"></param>
        /// <param name="v"></param>
        /// <param name="r"></param>
        /// <param name="g"></param>
        /// <param name="b"></param>
        private void hsvTorgb(double h, double s, double v, out double r, out double g, out double b)
        {
            r = 0;
            g = 0;
            b = 0;
            if (s == 0 && h == 0)
            {
                r = v; g = v; b = v;
            }
            if (h == 360)
            {
                h = 0;
            }
            h = h / 60;
            int i = (int)Math.Floor(h); //向下取整
            double f = h - i;
            double p = v * (1 - s);
            double q = v * (1 - (s * f));
            double t = v * (1 - (s * (1 - f)));
            switch (i)
            {
                case 0: { r = v; g = t; b = p; break; }
                case 1: { r = q; g = v; b = p; break; }
                case 2: { r = p; g = v; b = t; break; }
                case 3: { r = p; g = q; b = v; break; }
                case 4: { r = t; g = p; b = v; break; }
                case 5: { r = v; g = p; b = q; break; }
            }
            r = r * 255;
            g = g * 255;
            b = b * 255;
        }

        /// <summary>
        /// 触发事件
        /// 实时采集图像
        /// 并加入校正数据
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button11_Click(object sender, EventArgs e)
        {
            //画出下位机采集到的原始图像，没有经过任何处理
            //getdistanceandamplitudesorted(out dis);//dis存储的是采集到的深度数据
            //disp = datatranshalf(dis);//将一维的深度数据转换为2维数组（320*240）
            //sourceimg = drawSrcpic(disp);//将深度图像画出来    
            //pictureBox2.Image = sourceimg;

            //添加DRNU校正
            getdistanceandamplitudesorted(out dis);//输出一维数组，按照字节存储
            if (dis != null)
            {
                disp = datatranshalf(dis);//一维数组转换为二维数组，大小为【320,240】            
                disp1 = displace(disp);//[h=240,w=320]
                oneArry = srcDepthData(disp1);//二维转换为一维数组，类型为ushort

                caliprocess(oneArry.Length, oneArry);//调用杭州VR程序，在上位机进行数据DRNU处理

                disp2 = datatranshalf1to2(oneArry);//一维数组转换为二维数组，大小为【240,320】
                sourceimg = drawquaterpic(disp2);

                pictureBox2.Image = cvInitUndistortMap(sourceimg);

                label8.Text = "当前采集图像";

                iscollected = true;
            }
            else
            {
                iscollected = false;
            }
            
        }

        public Bitmap drawSrcpic(int[,] source)
        {
            int w = source.GetLength(0);
            int h = source.GetLength(1);
            int[,] a = new int[w, h];

            for (int i = 0; i < h; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    a[j, i] = source[j, i];
                }
            }
            Bitmap dst = new Bitmap(w, h);

            BitmapData dstdata = dst.LockBits(new Rectangle(0, 0, w, h),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            double r, g, b, H;

            unsafe
            {
                byte* p0 = (byte*)dstdata.Scan0;
                byte* p1;
                int stride = dstdata.Stride;
                for (int i = 0; i < h; i++)
                {
                    for (int j = 0; j < w; j++)
                    {
                        p1 = p0 + 3 * j + i * stride;
                        if (a[j, i] == 30000 || a[j, i] == 0)
                        {
                            p1[2] = p1[1] = p1[0] = 0;
                        }
                        else
                        {

                            H = sexiang(Convert.ToDouble(a[j, i]));
                            hsvTorgb(H, 1, 1, out r, out g, out b);
                            p1[2] = (byte)r;
                            p1[1] = (byte)g;
                            p1[0] = (byte)b;

                        }
                    }
                }
            }
            dst.UnlockBits(dstdata);

            return dst;
        }
        /// <summary>
        /// 触发事件
        /// 保存深度数据为csv文件，并保存深度图像为bmp格式
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button12_Click(object sender, EventArgs e)
        {
            if(disp == null)
            {
                MessageBox.Show("当前暂无可保存数据！", "Error");
            }
            else
            {
                DataTable dpdt2;
                string[] linename = new string[320];
                for (int i = 0; i < 320; i++)
                {
                    linename[i] = i.ToString();//给每一列取列名
                }

                dpdt2 = Convert2D(linename, disp); //320列，原始数据

                //string str;
                //SaveFileDialog sfdlg = new SaveFileDialog();

                //sfdlg.ShowDialog();
                //str = sfdlg.FileName;

                Bitmap tmp = (Bitmap)pictureBox1.Image;

                string csvpath = "C:\\Users\\quesx\\Desktop\\19.1.8\\" + System.DateTime.Now.Minute.ToString()
              + System.DateTime.Now.Second.ToString() +".csv";
                string picpath = "C:\\Users\\quesx\\Desktop\\19.1.8\\" + System.DateTime.Now.Minute.ToString()
              + System.DateTime.Now.Second.ToString()  +".bmp";
                //保存图像
                tmp.Save(picpath);
                //保存.csv文件
                SaveCSV(dpdt2, csvpath); 
            }
           
        }

        /// <summary>
        /// 功能函数
        /// 数组转化为DataTable数据表
        /// </summary>
        /// <param name="ColumnNames"></param>
        /// <param name="Arrays"></param>
        /// <returns></returns>
        public static DataTable Convert2D(string[] ColumnNames, int[,] Arrays)
        {
            //实例化DataTable数据表
            DataTable dt = new DataTable();

            foreach (string ColumnName in ColumnNames)
            {
                dt.Columns.Add(ColumnName, typeof(string));
            }

            for (int i1 = 0; i1 < Arrays.GetLength(1); i1++)
            {
                DataRow dr = dt.NewRow();
                for (int i = 0; i < ColumnNames.Length; i++)
                {
                    dr[i] = Arrays[i, i1].ToString();
                }
                dt.Rows.Add(dr);
            }

            return dt;
        }

        /// <summary>
        /// 将DataTable中数据写入到CSV文件中
        /// </summary>
        /// <param name="dt">提供保存数据的DataTable</param>
        /// <param name="fileName">CSV的文件路径</param>
        public static void SaveCSV(DataTable dt, string fullPath)
        {
            FileInfo fi = new FileInfo(fullPath);
            if (!fi.Directory.Exists)
            {
                fi.Directory.Create();
            }
            FileStream fs = new FileStream(fullPath, System.IO.FileMode.Create, System.IO.FileAccess.Write);
            //StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);
            StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.UTF8);
            string data = "";
            //写出列名称
            for (int i = 0; i < dt.Columns.Count; i++)
            {
                data += dt.Columns[i].ColumnName.ToString();
                if (i < dt.Columns.Count - 1)
                {
                    data += ",";
                }
            }
            sw.WriteLine(data);
            //写出各行数据
            for (int i = 0; i < dt.Rows.Count; i++)
            {
                data = "";
                for (int j = 0; j < dt.Columns.Count; j++)
                {
                    string str = dt.Rows[i][j].ToString();
                    str = str.Replace("\"", "\"\"");//替换英文冒号 英文冒号需要换成两个冒号
                    if (str.Contains(',') || str.Contains('"')
                        || str.Contains('\r') || str.Contains('\n')) //含逗号 冒号 换行符的需要放到引号中
                    {
                        str = string.Format("\"{0}\"", str);
                    }

                    data += str;
                    if (j < dt.Columns.Count - 1)
                    {
                        data += ",";
                    }
                }
                sw.WriteLine(data);
            }

            sw.Close();
            fs.Close();
            DialogResult result = MessageBox.Show("CSV文件保存成功！");
        }

        /// <summary>
        /// 触发事件
        /// 停止采集
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button15_Click(object sender, EventArgs e)
        {
            TestTimer.Enabled = false;

            pictureBox1.Image = null;
        }

       
        private void button16_Click(object sender, EventArgs e)
        {
            string str;
            SaveFileDialog sfdlg = new SaveFileDialog();

            sfdlg.ShowDialog();
            str = sfdlg.FileName;
            pictureBox1.Image.Save(str);//保存图像及命名string
        }
        
        /// <summary>
        /// 功能函数：根据坐标位置，显示所指连通域对应的连通域标记序列
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DepthInfoPic2(object sender, MouseEventArgs e)
        {
            if (connectAreaFlag)
            {
                if (newsignall[e.X, e.Y] <= max && newsignall[e.X, e.Y] > 0)
                {
                    label4.Text = string.Format("Position:({0},{1})  连通域:{2:f1}", e.X, e.Y, newsignall[e.X, e.Y]);
                }


            }
        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {

        }


    }
}
