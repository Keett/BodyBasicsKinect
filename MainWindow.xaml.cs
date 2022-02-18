//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// Çizilen el dairelerinin yarıçapı
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// Çizilen eklem çizgilerinin kalınlığı
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// Klip kenarı dikdörtgenlerinin kalınlığı
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// Kamera alanı noktalarının Z değerlerinin negatif olması için sabittir
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// Kapalı olarak takip edilen elleri çizmek için kullanılan Brush - fırça
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// Açık olarak takip edilen elleri çizmek için kullanılan Brush
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// Kement (işaretçi) konumunda izlenmekte olan elleri çizmek için kullanılan fırça
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// İzlenen eklemleri çizmek için kullanılan fırça
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// Halihazırda çıkarsanan eklemleri çizmek için kullanılan fırça
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// Halihazırda çıkarsanan kemikleri çizmek için kullanılan kalem
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// Vücut işleme çıktısı için çizim grubu
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// Görüntüleyeceğimiz çizim görüntüsü
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// Bir nokta türünü diğerine eşlemek için koordinat eşleyicisi (coordinate mapper)
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// Vücut frameleri için okuyucu
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// Vücut için dizi
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// kemiklerin tanımı
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// Ekran genişliği (derinlik alanı)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// Ekran yüksekliği (derinlik alanı)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// İzlenen her vücut için renk listesi
        /// </summary>
        private List<Pen> bodyColors;


        private Point kneeLeftPoint = new Point();
        private Point hipLeftPoint = new Point();
        private Point ankleLeftPoint = new Point();
        private Point neckPoint = new Point();
        private Point headPoint = new Point();


        /// <summary>
        /// Current status text to display
        /// Görüntülenecek geçerli durum metni
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// MainWindow sınıfının yeni bir örneğini başlatır.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            // Halihazırda bir sensör desteklemesi
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            // Koordinat mapper'ı al 
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            // derinlik (ekran) kapsamını elde edin
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            // ortak alan boyutunu al
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            // vücut framleri için okuyucuyu aç
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            // iki eklem arasındaki çizgi olarak tanımlanan bir kemik
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso -- Gövde
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            // her BodyIndex için bir tane olmak üzere vücut renklerini doldurma (birden fazla kişi için farklı renkte body çizimi için)
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier -- olay bildiricisi
            // sensörün kullanılamaz hale geldiği olayı ele alır (duraklatıldı, kapatıldı, fişi çekildiği durumlar)
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            //Sensor kullanılıp kullanılmadığı durum bilgisi
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            // Çizim için kullanacağımız çizim grubunu oluşturun
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            // Görüntü kontrolümüzde kullanabileceğimiz bir görüntü kaynağı oluşturun
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            // bu basit örnekte pencere nesnesini görünüm modeli olarak kullanın
            this.DataContext = this;

            // initialize the components (controls) of the window
            // pencerenin bileşenlerini (kontrollerini) başlat
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// Pencere denetimlerinin değiştirilebilir verilere bağlanmasına izin vermek için INotifyPropertyChangedPropertyChanged olayı
        /// InotifyPropertyChanged'den implement edilen event
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// Görüntülenecek bitmap'i alır
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// Görüntülenecek geçerli durum metnini alır veya ayarlar
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    // metnin değiştiği tüm bağlı öğeleri bildir
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// Başlatma görevlerini yürütün
        /// </summary>
        /// <param name="sender">object sending the event/ olayı gönderen nesne</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// Kapatma görevlerini yürütün
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// Sensörden gelen vücut frame verilerini işler
        /// (BodyFrameArrivedEventArgs e): bir vücut frame'i okuyucusunun FrameArrived olayı için bağımsız değişkenleri temsil eder.
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false; //veri alındı mı?

            // BodyFrame : sensörün görüş alanında olan kişiler hakkında tüm hesaplanmış gerçek zamanlı izleme bilgilerini içeren bir frame'i temsil eder.
            // AcquireFrame() : Bu referans tarafından tutulan çerçeveyi alır
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    // GetAndRefreshBodyData ilk kez çağrıldığında, Kinect dizideki her bir Body'i tahsis edecektir.
                    // Bu body nesneleri atılmadığı ve dizide null değerine ayarlanmadığı sürece, bu body nesneleri yeniden kullanılacaktır.

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                // DrawingContext : Draw, Push ve Pop komutlarını kullanarak görsel içeriği açıklar.
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    // Oluşturma (Render) boyutunu ayarlamak için şeffaf bir arka plan çizin - Arkaplan rengive boyutunu ayarlama
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    // Body: Tek vücudu temsil eder
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        // IsTracked : vücudun takip edilip edilmediğini alır
                        if (body.IsTracked)
                        {
                            // DrawClippedEdges : hangi kenarların vücut verilerini kırptığını göstermek için göstergeler çizer
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            // convert the joint points to depth (display) space
                            // eklem noktalarını derinlik (görüntü) alanına dönüştür

                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            // Bütün jointType larda gez, kamera pozisyonlarını al, pozisyon Z sabitiyle çarp,  jointPoints[ilgili jointType] = yeni pozisyon değerlerini ata.
                            // JointType kadar döner ve yenilenir.
                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinate mapper from returning (-Infinity, -Infinity)
                                // bazen çıkarsanan bir eklemin derinliği(Z) negatif olarak gösterilebilir
                                // Koordinat mapper dönüşüne engel olmak için 0,1f'ye kadar sıkıştırın (-sonsuz, -sonsuz)

                                CameraSpacePoint position = joints[jointType].Position;
                                // CameraSpacePoint : Kamera alanında 3 boyutlu bir noktayı temsil eder. Koordinat sisteminin başlangıç ​​noktası (0,0,0) kamera konumudur.
                                // position : Eklemin kamera boşluğundaki konumu. (Kamera Konum)

                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                // DepthSpacePoint : derinlik uzayında bir 2d noktayı temsil eder
                                // MapCameraPointToDepthSpace : bir noktayı kamera uzayından derinlik uzayına eşler.(Gerçek Konum)

                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            SetRealTimePosition(jointPoints);

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    // i̇şleme (render) alanımızın dışına çizilmeyi önleme
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Gerçek zamanlı pozisyon konumlarını al
        /// </summary>
        /// <param name="jointPoints"></param>
        public void SetRealTimePosition(Dictionary<JointType,Point> jointPoints)
        {
            foreach (KeyValuePair<JointType, Point> veri in jointPoints)
            {
                switch (veri.Key)
                {
                    case JointType.Neck:
                        break;
                    case JointType.Head:
                        break;
                    case JointType.ShoulderLeft:
                        break;
                    case JointType.ElbowLeft:
                        break;
                    case JointType.WristLeft:
                        break;
                    case JointType.ShoulderRight:
                        break;
                    case JointType.ElbowRight:
                        break;
                    case JointType.WristRight:
                        break;
                    case JointType.HipLeft:
                        hipLeftPoint = veri.Value;
                        break;
                    case JointType.KneeLeft:
                        kneeLeftPoint = veri.Value;
                        break;
                    case JointType.AnkleLeft:
                        ankleLeftPoint = veri.Value;
                        break;
                    case JointType.FootLeft:
                        break;
                    case JointType.HipRight:
                        break;
                    case JointType.KneeRight:
                        break;
                    case JointType.AnkleRight:
                        break;
                    case JointType.FootRight:
                        break;
                    default:
                        break;
                }
            }
            GetKinematik();
        }

        public void GetKinematik()
        {
            double ankleLeftAngle = getAngle(kneeLeftPoint, hipLeftPoint, ankleLeftPoint);
            Console.WriteLine("Ankle Left Angle : {0}",ankleLeftAngle);
        }

        /// <summary>
        /// Üç noktası bilinen koordinatların açısal değerini bulan fonksiyon
        /// </summary>
        /// <param name="startPoint">Başlangıç noktası aynı zamanda açısı bulunacak eklem</param>
        /// <param name="secondPoint">Başlangıç noktasına bağlı diğer eklem</param>
        /// <param name="thirdPoint">Başlangıç noktasına bağlı diğer eklem</param>
        /// <returns></returns>
        double getAngle(Point startPoint, Point secondPoint, Point thirdPoint)
        {
            Point startToSecondPoint = GetPointToVector(startPoint, secondPoint);
            Point startToThirdPoint = GetPointToVector(startPoint, thirdPoint);
            double skalerCarpim = DotProduct(startToSecondPoint, startToThirdPoint);
            double vectorLength = VectorLength(startToSecondPoint) * VectorLength(startToThirdPoint);
            return skalerCarpim / vectorLength * 180 / Math.PI;
        }

        /// <summary>
        /// Vücut üye (segment) açı hesaplaması
        /// </summary>
        /// <param name="startPoint">Başlangıç koordinatı</param>
        /// <param name="lastPoint">Bitiş Koordinatı</param>
        /// <returns></returns>
        double Slope(Point startPoint, Point lastPoint) => Math.Atan((lastPoint.Y - startPoint.Y) / (lastPoint.X - startPoint.X));

        /// <summary>
        /// İki noktayı tek vektör noktasına çeviren fonksiyon
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="lastPoint"></param>
        /// <returns>Vectör cinsinden Point türünde koordinat verisi</returns>
        public Point GetPointToVector(Point startPoint, Point lastPoint)
        {
            Point result = new Point();
            result.X = lastPoint.X - startPoint.X;
            result.Y = lastPoint.Y - startPoint.Y;
            return result;
        }

        /// <summary>
        /// İki vektörün skaler çarpım hesaplaması
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="lastPoint"></param>
        /// <returns></returns>
        double DotProduct(Point startPoint, Point lastPoint) => (startPoint.X) * (lastPoint.X) + (startPoint.Y) * lastPoint.Y;

        /// <summary>
        /// Koordinatı bilinen vektörün uzunluğu
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        double VectorLength(Point point) => Math.Sqrt(Math.Pow(point.X, 2) + Math.Pow(point.Y, 2));
        
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                // TrackingState : bir body'nin veya body'nin özniteliğinin izlenme durumunu belirtir.
                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked) // izlenen
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred) // çıkarsanan
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            // BOTH (Her iki eklem) eklemleri izlenmediği sürece tüm çizilmiş kemiklerin çıkarıldığını varsayıyoruz.
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// Hangi kenarların gövde verilerini kırptığını göstermek için göstergeler çizer
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// Sensörün kullanılamaması durumunu ele alır (Örn. duraklatıldı, kapatıldı, fişe takılı değil).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
