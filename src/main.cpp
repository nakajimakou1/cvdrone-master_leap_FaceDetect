/*
  CVDrone + LeapMotion 2014.01.25
  
    - puku0x, CV Drone (= OpenCV + AR.Drone)
    https://github.com/puku0x/cvdrone, Accessed on 25 Jan 2014.

*/

#include "ardrone/ardrone.h"
#include "Leap.h"

#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")

#include <OpenCV/cxcore.h>
#define snprintf _snprintf

using namespace cv;

//マウスイベント用
//http://ameblo.jp/banquet-of-merry-widow/entry-11101618791.html
typedef struct MouseParam{
	unsigned int x;
	unsigned int y;
	int event;
	int flags;
} MouseParam;

// MouseMode :
//       0:Mouse Up Reset
//       1:Mouse Down
// MouseCtrlMode :
//       0:Ctrl OFF
//       1:Ctrl On
int MouseMode = 0;
int MouseCtrlMode = 0;
int preX=0, preY=0;
bool MouseARMode = false;


class pLeapData
{
public:
	float pitch;		//前p：-0.5　後p： 0.9
	float yaw;		//左y：-1.0　右y： 0.7
	float roll;		//左R： 0.8　右R：-1.0

	float PosX;		//左右　　　左  -150　～　右 150
	float PosY;		//上下昇降　下    50　～　上 300
	float PosZ;		//前後　    手前-100　～　奥 100

	float PosX_pre;		//左右　　　左  -150　～　右 150
	float PosY_pre;		//上下昇降　下    50　～　上 300
	float PosZ_pre;		//前後　    手前-100　～　奥 100

	bool mLeapDebugPrint;
	bool mLeapMode;

	void init(void );

//private:

};
void pLeapData::init()
{
		pitch = 0;		//前p：-0.5　後p： 0.9
		yaw   = 0;		//左y：-1.0　右y： 0.7
		roll  = 0;		//左R： 0.8　右R：-1.0

		PosX  = 0;		//左右　　　左  -150　～　右 150
		PosY  = 0;		//上下昇降　下    50　～　上 300
		PosZ  = 0;		//前後　    手前-100　～　奥 100

		PosX_pre = 0;	//左右　　　左  -150　～　右 150
		PosY_pre = 0;	//上下昇降　下    50　～　上 300
		PosZ_pre = 0;	//前後　    手前-100　～　奥 100

		mLeapDebugPrint = false;
		mLeapMode = false;
}


// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
void LeapValueUpdate(pLeapData,Leap::Controller);
int initdrone(ARDrone* );
void imgSave(char *, IplImage *);

void PlayWaveSound(void);
void mMouseEventfunc(int , int , int , int , void  *);

//std::string TextRecogReq(std::string);
//char *TextRecogReq(char *);

void gotoPlaySound(double , double, double, int, int);

//static bool mLeapDebugPrint = true;
bool mNonDronDebug = false;	// true:ArDroneなしのデバッグ
//bool mNonDronRDebug = true;	// true:ArDroneへのTakeOff　Moveコマンドは出力しない。
bool mNonDronRDebug = false;	// true:ArDroneへのTakeOff　Moveコマンドは出力しない。
bool mMouseDebugPrint = false;	// true:Mouse Move デバッグ出力する。

bool mFaceDetectMode = false;
bool mFaceLostFlag = false;

bool mArDroneCommandFlag = false;

double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;

// AR.Drone class
ARDrone ardrone;

#include <time.h>


int main(int argc, char **argv)
{
	//int i;
	//static IplImage *src_img = 0, *src_gray = 0;
	CascadeClassifier face_cascade;
	//OK 2014.02.14 精度は荒いが速度はよい　※速度重視
	face_cascade.load("..\\..\\data\\haarcascade_frontalface_alt2.xml");
	
	//setup image files used in the capture process
	Mat captureFrame;
	Mat grayscaleFrame;

	static pLeapData pLeapData;
	pLeapData.init();

	static bool mLeapnot = false;
	static bool mTakOffFlag = false;
	static bool mSendCommandflag = false;
	static int  mSendCommandcounter = 0;
	static int  mSoundCommandcounter = 0;
	static int  mSoundCommandOKcounter = 20;
	
	float pitch = 0;		//前p：-0.5　後p： 0.9
	float yaw   = 0;		//左y：-1.0　右y： 0.7
	float roll  = 0;		//左R： 0.8　右R：-1.0
	float pitch_pre = 0;	//前p：-0.5　後p： 0.9
	float yaw_pre  = 0;		//左y：-1.0　右y： 0.7
	float roll_pre = 0;		//左R： 0.8　右R：-1.0

	float PosX = 0;			//左右　　　左  -150　～　右 150
	float PosY = 0;			//上下昇降　下    50　～　上 300
	float PosZ = 0;			//前後　    手前-100　～　奥 100
	float PosX_pre = 0;		//左右　　　左  -150　～　右 150
	float PosY_pre = 0;		//上下昇降　下    50　～　上 300
	float PosZ_pre = 0;		//前後　    手前-100　～　奥 100

	float Para_pre = 0.80f;	//
	float Para_cur = 0.2f;	//

	Leap::Frame frame;		// controller is a Leap::Controller object
	Leap::HandList hands;
	Leap::Hand firstHand;

	//double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;

	int mbatValue = 0;

	//マウスイベント用
	//http://ameblo.jp/banquet-of-merry-widow/entry-11101618791.html
	MouseParam mparam;
	mparam.x = 0; mparam.y = 0; mparam.event = 0; mparam.flags = 0;
	//ウインドウへコールバック関数とコールバック関数からイベント情報を受け取る変数を渡す。
	//setMouseCallback( wname, &mfunc, &mparam );

	// AR.Drone class
    // ARDrone ardrone;

	if(mNonDronDebug == true)
	{
	}else
	{   
		// Initialize
		//if (!ardrone.open()) {
		if ( initdrone(&ardrone) == -1) {
			printf("Failed to initialize.\n");
			return -1;
		}
	}

#ifdef MCISOUND
	PlayWaveSound();
#endif

    // Battery
    printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

    // Instructions
    printf("***************************************\n");
    printf("*       CV Drone sample program       *\n");
    printf("*           - How to Play -           *\n");
    printf("***************************************\n");
    printf("*                                     *\n");
    printf("* - Controls -                        *\n");
    printf("*    'Space' -- Takeoff/Landing       *\n");
    printf("*    'Up'    -- Move forward          *\n");
    printf("*    'Down'  -- Move backward         *\n");
    printf("*    'Left'  -- Turn left             *\n");
    printf("*    'Right' -- Turn right            *\n");
    printf("*    'Q'     -- Move upward           *\n");
    printf("*    'A'     -- Move downward         *\n");
    printf("*                                     *\n");
    printf("* - Others -                          *\n");
    printf("*    'C'     -- Change camera         *\n");
    printf("*    'Esc'   -- Exit                  *\n");
    printf("*                                     *\n");
    printf("*    'F' --mFaceDetectMode：スイッチ  *\n");
    printf("*    'L' --LeapMode：スイッチ         *\n");
    printf("*                                     *\n");
    printf("***************************************\n\n");
	//　
	//2014.01.15 add
	Leap::Controller leapController;
        
	// Get an image
	static IplImage *image;
	//ardrone.setCamera(0);
	ardrone.setCamera(1);//下面カメラ指定

	//顔検出後の枠用
	CvPoint pt1;
	pt1.x = 100;
	pt1.y = 100;
	CvScalar rcolor;
	rcolor = CV_RGB( 128,  80, 128);

	//ウィンドウの表示
	cvNamedWindow ("FaceDetectW", CV_WINDOW_AUTOSIZE);
	cvNamedWindow ("camera", CV_WINDOW_AUTOSIZE);
	//ウインドウへコールバック関数とコールバック関数からイベント情報を受け取る変数を渡す。
	cvSetMouseCallback( "camera", &mMouseEventfunc, &mparam );

	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);

    while (1) 
	{
        // Key input
        int key = cvWaitKey(33);
        //int key = cvWaitKey(15);
		if (key == 0x1b){
			break;
		}

		//2014.03.09 add
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;
		vr = 0.0;

		//音声出力タイミング用ワーク
		if (mSendCommandflag == true)
		{
			if(mSendCommandcounter++ > 50)
			{
				mSendCommandflag = false;
				mSendCommandcounter = 0;
			}
		}
        // Update
		if(mNonDronDebug == false)
		{
	        if (!ardrone.update()) 
				break;

			// Get an image
			image = ardrone.getImage();
			if((mbatValue = ardrone.getBatteryPercentage()) < 30){
				printf("Battery = %d%%\n",mbatValue );

				if(mArDroneCommandFlag == false)
					ardrone.move3D(0.0, 0.0, 0.0, 0.0);
				msleep(80);
				ardrone.landing();
				printf("Landing\n");
				msleep(180);
			}
		//}
	
#ifndef FACEDETECT
		try{
			//2014.02.15　FaceDetection追加
			// (3)メモリを確保し，読み込んだ画像のグレースケール化，ヒストグラムの均一化を行う
			CvMemStorage *storage = 0;
			storage = cvCreateMemStorage (0);
			cvClearMemStorage (storage);

			//Mat captureFrame;
			//Mat grayscaleFrame;
			Mat captureFrameMat = cvarrToMat(image);
			cvtColor(captureFrameMat, grayscaleFrame, CV_BGR2GRAY);
			equalizeHist(grayscaleFrame, grayscaleFrame);
 
			//　mFaceDetectMode：Fキーにてスイッチ
			if((mFaceDetectMode == true)
				&&((ardrone.getCameraMode() == 0)||(ardrone.getCameraMode() == 2)))//正面カメラの場合に有効
			{
				// (4)物体（顔）検出
				//create a vector array to store the face found
				std::vector<Rect> faces;
				face_cascade.detectMultiScale(grayscaleFrame, faces, 1.2, 4, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(30,30));
				//printf("FaceNum:%02d\n",faces.size());

				// (5)検出された全ての顔位置に，四角を描画する
				Point pt1;
				Point pt2;
				Point cPt1;//Center Mark
				int mFaceHeight=0;
				int mFaceWidth=0;
				//複数検出の場合は、最大のものをTrackingする。
				for(int i = 0; i < (signed)faces.size(); i++)
				{
					if(i==0)
					{
						pt1.x = faces[i].x + faces[i].width;
						pt1.y = faces[i].y + faces[i].height;
						mFaceHeight = faces[i].height;
						mFaceWidth = faces[i].width;
						pt2.x = faces[i].x ;
						pt2.y = faces[i].y ;
						cPt1.x = faces[i].x + faces[i].width/2;
						cPt1.y = faces[i].y + faces[i].height/2;
					}else
					{
						//最大の検出対象の値をキープ
						if(faces[i-1].height < faces[i].height)
						{
							pt1.x = faces[i].x + faces[i].width;
							pt1.y = faces[i].y + faces[i].height;
							mFaceHeight = faces[i].height;
							mFaceWidth = faces[i].width;
							pt2.x = faces[i].x;
							pt2.y = faces[i].y;
							cPt1.x = faces[i].x + faces[i].width/2;
							cPt1.y = faces[i].y + faces[i].height/2;
						}
					}
				}

				//printf("FaceNum:%02d",faces.size());
				if(faces.size() > 0)
				{
					//顔検出した場合の処理
					mFaceLostFlag = false;

					rectangle(captureFrameMat, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);
					//Center Mark
					circle (captureFrameMat,cPt1,5,rcolor,-2);

					//double vx=0.0, vy=0.0, vr=0.0, vz=0.0;
					if((cPt1.x > 0)&&(cPt1.x < 200)){
						vr = 1.0;
					}
					if((cPt1.x > 280)&&(cPt1.x <350)){
						vr = 1.0;
					}
					if((cPt1.x > 450)&&(cPt1.x < 520)){
						vr = -1.0;
					}
					if((cPt1.x > 600)&&(cPt1.x < 800)){
						vr = -1.0;
					}
					
					if((cPt1.y > 0)&&(cPt1.y < 350)){
						//vz = 0.75;
					}else if((cPt1.y > 400)&&(cPt1.y < 600)){
						//vz = -0.75;
					}
					if((mFaceHeight > 1)&&(mFaceHeight < 200)){
						//vx = 0.75;
						//vz = 0.75;
					}else if((mFaceHeight > 300)&&(mFaceHeight < 600)){
						//vx = -0.75;
						//vz = -0.75;
					} 

					if(!mNonDronRDebug)
					{
						if((!ardrone.onGround())&&(mArDroneCommandFlag == false))
						{
							//time_t now = time(NULL);
							//struct tm *pnow = localtime(&now);
							//printf("FT:%02d:%02d:%02d X:%03d Y:%03d vx:%02.1f vy:%02.1f vz:%02.1f vr:%02.1f FH:%03d\n",pnow->tm_hour,pnow->tm_min,pnow->tm_sec, cPt1.x,cPt1.y, vx, vy, vz, vr, mFaceHeight);
							//ardrone.move3D(vx, vy, vz, vr);
							//msleep(30);
						}
					}
				}else
				{
					//Face　Lostモード
					if(mFaceLostFlag == false)
					{
						mFaceLostFlag = true;
						if(!mNonDronRDebug)
						{
							if (!ardrone.onGround())
							{
								if(mArDroneCommandFlag == false)
								{
									//ardrone.move3D(0.0, 0.0, 0.0, 0.0);
									//printf(" X:%03d Y:%03d vx:%02d vy:%02d vz:%02d vr:%02d FH:%03d\n", 0, 0, 0, 0, 0, 0, 0);
									printf("Face　Lostモード\n");
									//msleep(100);
								}
							}
						}
					}
				}

				//2014.02.22
				// height value enable 150 - 400
				// x 150 - 600  center:400
				// y 150 - 600  center:400
				//printf(" x:%02d y:%02d w:%02d h:%02d",faces[0].x,faces[0].y,faces[0].width,faces[0].height);
				//printf(" cx:%02d cy:%02d w:%02d h:%02d",cPt1.x,cPt1.y,faces[0].width,faces[0].height);
				//printf("\n");
				//
				IplImage wimage = captureFrameMat;
				//static IplImage wimage = grayscaleFrame;
				//cvCopy( image, wimage);
				image = &wimage;
			}

		}catch(char *e)
		{
			printf("%s\n",e);
		}
#endif
		}

        //2014.03.09 add
		if((mLeapnot != true)&&(pLeapData.mLeapMode == true)&&(leapController.isConnected()))
		{
			frame = leapController.frame(); // controller is a Leap::Controller object
			hands = frame.hands();
			firstHand = hands[0];

			pitch_pre = pitch;
			pitch = firstHand.direction().pitch();//前p：-0.5　後p： 0.9
			pitch = pitch_pre*Para_pre + pitch*Para_cur;	//Para_pre:0.80 Para_cur:0.20

			yaw_pre = yaw;    //左y：-1.0　右y： 0.7
			yaw = firstHand.direction().yaw();    //左y：-1.0　右y： 0.7
			yaw = yaw_pre*Para_pre + yaw*Para_cur;

			roll_pre = roll; //左R： 0.8　右R：-1.0
			roll = firstHand.palmNormal().roll(); //左R： 0.8　右R：-1.0
			roll = roll_pre*Para_pre + roll*Para_cur;

			PosX = frame.pointables().leftmost().tipPosition().x;       //左右　　　左-150　～　右 150
			PosY = frame.pointables().leftmost().tipPosition().y;       //上下昇降　下  50　～　上 300
			PosZ = frame.pointables().leftmost().tipPosition().z * (1); //前後　  手前-100　～　奥 100

			if(pLeapData.mLeapDebugPrint == true){
				printf("%03d XYZ:%03.02f:%03.02f:%03.02f p:%03.02f y:%03.02f r:%03.02f TF:%01i: %i\n",mSoundCommandcounter, PosX,PosY,PosZ,pitch,yaw,roll,(int)mTakOffFlag,mSendCommandcounter);
			}

			//LeapMotion Value set
			//LeapMotionに近づけると TakeOFF
			if((PosY > 50) && (PosY < 75) && (mTakOffFlag == true))
			{
				if(mNonDronDebug == false)
				{
					if (ardrone.onGround())
					{
						mTakOffFlag = false;
					}else
					{
						if(mSoundCommandcounter>mSoundCommandOKcounter){
							sndPlaySound("..\\..\\src\\resource\\HackathonUser1orimasu.wav", SND_ASYNC);//orimasu
							mSoundCommandcounter = 0;
						}
						
						if(!mNonDronRDebug) {
							ardrone.landing();
						}

						mTakOffFlag = false;
						mSendCommandflag = true;

						if(!mNonDronRDebug) 
							if((pLeapData.mLeapMode == true)&&(mArDroneCommandFlag == false))
								ardrone.move3D(0.0, 0.0, 0.0, 0.0);

						msleep(250);
					}
				}
			}

			if((PosY > 200) && (PosY < 250) && (mTakOffFlag == false))
			{
				if(mNonDronDebug == false)
				{
					if (ardrone.onGround())
					{
						if(pLeapData.mLeapMode == true)
						{
							mSendCommandflag = true;
							if(!mNonDronRDebug) 
								ardrone.takeoff();

							msleep(250);
						
							printf("Leap takeoff\n");
							mTakOffFlag = true;
							ardrone.move3D(0.0, 0.0, 0.0, 0.0);
							//msleep(50);
							msleep(100);
						}
					}
				}
			}

			if((pitch > -0.6) && (pitch < -0.45)){
			//前進
				//vx =  1.0;
			}else if((pitch < 0.9)&&(pitch > 0.5)){
			//back
				//vx =  -1.0;
			}

			if((roll > 0.5)&&(roll < 0.8)){
			//左傾斜
				//vy = 1.0;
			}else if((roll < -1.0)&&(roll > -1.4)){
			//右傾斜
				//vy = -1.0;
			}else if((vx == 0) &&(vy == 0))
			{
				//左向き
				if((yaw < -0.5)&&(yaw > -0.8)){
					vr = 1.0;
				}
				//右向き
				if((yaw > 0.05)&&(yaw < 0.5)){
					vr = -1.0;
				}
			}else
			{
				vr = 0.0;
			}

			if(!pLeapData.mLeapDebugPrint == true){
				printf("vxyr:%02.01f %02.01f %02.01f: %02.01f %02.01f \n",vx,vy,vr,roll,roll_pre);
			}
		}//(mLeapnot != true)

		//キーコントロール入力
        // Take off / Landing 
		if(mNonDronDebug == false)
		{
			if (key == ' ') 
			{
				if (ardrone.onGround())
				{
					if(!mNonDronRDebug) 
						ardrone.takeoff();

					msleep(300);
					printf("takeoff\n");
					if(mArDroneCommandFlag == false)
					{
						ardrone.move3D(0.0, 0.0, 0.0, 0.0);
						msleep(200);
					}

					if(mSoundCommandcounter>mSoundCommandOKcounter)
					{
						sndPlaySound("..\\..\\src\\resource\\HackathonUser1tobimasu.wav", SND_ASYNC);//orimasu
						mSoundCommandcounter = 0;
					}

					mTakOffFlag = true;
					mSendCommandflag = true;
					//msleep(500);
				}else//
				{
					if(!mNonDronRDebug)// false
					{
						ardrone.landing();
						printf("Landing\n");
						msleep(500);
						ardrone.move3D(0.0, 0.0, 0.0, 0.0);
						msleep(200);
					}

					if(mSoundCommandcounter>mSoundCommandOKcounter)
					{
						sndPlaySound("..\\..\\src\\resource\\HackathonUser1orimasu.wav", SND_ASYNC);//orimasu
						mSoundCommandcounter = 0;
					}

					mTakOffFlag = false;
					mSendCommandflag = true;
					//msleep(500);
				}
			}//'Space'
		}

		//printf("*    'Space' -- Takeoff/Landing       *\n");
		//printf("*    'Up'    -- Move forward          *\n");
		//printf("*    'Down'  -- Move backward         *\n");
		//printf("*    'Left'  -- Turn left             *\n");
		//printf("*    'Right' -- Turn right            *\n");
		//printf("*    'Q'     -- Move upward           *\n");
		//printf("*    'A'     -- Move downward         *\n");
        // Move
        //vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 0x260000) vx =  1.0;//Up arrow
        if (key == 0x280000) vx = -1.0;//Down arrow key
        if (key == 0x250000) vr =  1.0;//Left arrow key
        if (key == 0x270000) vr = -1.0;//Right arrow key
        if (key == 'q')      vz =  1.0;
        if (key == 'a')      vz = -1.0;

        if (key == 'r')
		{
			//Reset
			//ardrone.emergency();
			if(mNonDronDebug == true)
			{
			}else
			{   
				//2014.03.09 add
				vx = 0.0;
				vy = 0.0;
				vz = 0.0;
				vr = 0.0;
				//ardrone.close();
				if ( initdrone(&ardrone) == -1) {
					printf("Failed to initialize.\n");
					return -1;
				}
			}
		}
		
		// 2014.03.02 add
        if((key == 'f')||(key == 'F')){
			mFaceDetectMode = !mFaceDetectMode;
			printf("Face Mode:%02X  Battery = %d%%\n",mFaceDetectMode, ardrone.getBatteryPercentage());
		}
		
		if((key == 'l')||(key == 'L')){
			pLeapData.mLeapMode = !pLeapData.mLeapMode;
			printf("Leap Mode:%02X  Battery = %d%%\n",pLeapData.mLeapMode, ardrone.getBatteryPercentage());
		}

		if((key == 'v')||(key == 'V')){
			printf("Btry:%d%% mSendCommandflag:%02d\n", ardrone.getBatteryPercentage(), mSendCommandflag);
		}

		if((key == '0')||(key == '0')){
			printf("Btry:%d%% reset setFlatTrim():%02d\n", ardrone.getBatteryPercentage(), key);
			ardrone.setFlatTrim();
			msleep(500);
		}


		if(mNonDronDebug == false)
		{
			gotoPlaySound(vx, vy, vr, mSoundCommandcounter, mSoundCommandOKcounter);
			if((!mNonDronRDebug)&&(!ardrone.onGround()))
			{
				if((mArDroneCommandFlag == false)&&(MouseARMode == false))
				{
					ardrone.move3D(vx, vy, vz, vr);
					//ardrone.move3D(vx, vy, vz, vr);
					//ardrone.move3D(vx,  vr, 0.0, vy);
					msleep(150);
					time_t now = time(NULL);
					struct tm *pnow = localtime(&now);
					printf("KLT:%02d:%02d:%02d vx:%02.1f vy:%02.1f vz:%02.1f vr:%02.1f \n",pnow->tm_hour,pnow->tm_min,pnow->tm_sec, vx, vy, vz, vr);
				}
			}
			//ardrone.move3D(0, 0, 0, 0);
		}

        // Change camera
		if(mNonDronDebug == false)
		{
	        static int mode = 0;
		    if((key == 'c')||(key == 'C')) 
				ardrone.setCamera(++mode%2);
				//ardrone.setCamera(++mode%4);

	        // Display the image
			//cvCircle (image,pt1,30,rcolor,2);
			IplImage wGrayImage = grayscaleFrame;
			cvShowImage ("FaceDetectW", &wGrayImage);             
		    cvShowImage("camera", image);

			cvMoveWindow( "FaceDetectW", 600, 0 );
			cvMoveWindow( "camera", 50, 0 );
			//WindowFromPoint(point(100,200));
		
			if ((key == 's') && (ardrone.getCameraMode() == 1))
			{
				imgSave("..\\..\\SaveFileName.jpg", &wGrayImage);
			}
		}
    }//while loop

    // See you
	if(mNonDronDebug == false)
	{
	    ardrone.close();
		cvDestroyWindow ("camera");
		cvDestroyWindow ("FaceDetectW");
	}

    return 0;
}


void gotoPlaySound(double vx, double vy, double vr, int mSoundCommandcounter , int mSoundCommandOKcounter)
{
	if((vx == 1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1maenitobimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}
	if((vx == -1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1bakkusimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}
	if((vy == 1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1hidarinimawarimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}
	if((vy == -1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1miginimawarimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}

	if((vr == 1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1hidarinitobimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}
	if((vr == -1)&&(mSoundCommandcounter>mSoundCommandOKcounter)){
		sndPlaySound("C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1miginitobimasu.wav", SND_ASYNC);//orimasu
		mSoundCommandcounter = 0;
	}
}

#ifdef MCISOUND
//static void PlayWaveSound(String SoundFile)
static void PlayWaveSound(void)
{
	//ファイルを開く
mciSendString("open C:\\cvdrone-master_leap\\src\\resource\\HackathonUser1tobimasu.wav alias test",NULL,0,NULL);

//ファイルを実行する
mciSendString("seek test to start",NULL,0,NULL);
mciSendString("play test",NULL,0,NULL);

//実行したファイルを止める
mciSendString("stop test",NULL,0,NULL);

//ファイルを閉じる
mciSendString("close test",NULL,0,NULL);
}

#endif


int initdrone(ARDrone *ardrone)
{
	// Initialize
	if (!(*ardrone).open()) {
		//printf("Failed to initialize.\n");

		//2014.03.08 add
		//(*ardrone).setFlatTrim();
		return -1;
	}
	return 0;
}

void imgSave(char *SaveFileName, IplImage *image)
{
	//画像ファイルに保存
    //imwrite(SaveFileName, image); 
	//NG cvSaveImage(SaveFileName, image->imageData);
	cvSaveImage(SaveFileName, image);
	return;
}


void goMousePoint(int x, int y, int MouseCtrlMode)
{
	//printf("cmode:%02d vx:%02d vy:%02d \n",ardrone.getCameraMode(),vx,vy);

	if(!mNonDronRDebug)
	{
		double vx=0.0, vy=0.0, vr=0.0, vz=0.0;

		if (!ardrone.onGround())
		{
			//CameraMode 0:正面カメラ
			//           1:下面カメラ
			//           2:正面カメラ + 小　下面カメラ
			//           3:下面カメラ + 小　正面カメラ

			int threshValue = 300;
			//正面カメラ
			if((ardrone.getCameraMode() == 0)||(ardrone.getCameraMode() == 2))
			{
				//printf("cmode:%02d \n",ardrone.getCameraMode());
				if((x>0)&&(x<800)&&(y>0)&&(y<600))
				{
					if((!MouseCtrlMode))
					{
						if((x>0)&&(x < 300)){
							//vy = -1.5;
						}
						if((x>500)&&(x < 800)){
							//vy = 1.5;
						}
						if((y>0)&&(y < 150)){
							vz = 1.0;
						}
						if((y>450)&&(y < 600)){
							vz = -1.0;
						}

						/*
						if(((x - preX) > 5)&&((x - preX) < threshValue)){
							vy = -1;
						}
						if(((x - preX) < -5)&&((x - preX) > (-1 * threshValue))){
							vy = 1;
						}
						if(((y - preY) < 10)&&((y - preY) > -10)){
							if((y>0)&&(y < 150)){
								vz = 1;
							}
							if((y>450)&&(y < 600)){
								vz = -1;
							}
						}

						if(((y - preY) > 5)&&((y - preY) < threshValue/2)){
							//画面下へドラッグ
							//vz = -1;
						}
						if(((y - preY) < -5)&&((y - preY) > (-1 * threshValue/2))){
							//vz = 1;
						}
						*/

					}else{
						//マルチタッチモード
						//前後飛行
						if(((x - preX) > 5)&&((x - preX) < threshValue)){
							vz = 1.0;
						}
						if(((x - preX) < -5)&&((x - preX) > (-1 * threshValue))){
							vz = -1.0;
						}
					}
				}
			}

			//下面カメラ
			if((ardrone.getCameraMode() == 1)||(ardrone.getCameraMode() == 3))
			{
				//printf("cmode:%02d \n",ardrone.getCameraMode());
				if((x>0)&&(x<800)&&(y>0)&&(y<600))
				{
					if((!MouseCtrlMode))
					{
						if((x>0)&&(x < 300)){
							vy = 1.5;
						}
						if((x>500)&&(x < 800)){
							vy = -1.5;
						}
						if((y>0)&&(y < 200)){
							vx = 1.5;
						}
						if((y>400)&&(y < 600)){
							vx = -1.5;
						}
						/*
						if(((x - preX) > 5)&&((x - preX) < threshValue)){
							vy = -1;
						}
						if(((x - preX) < -5)&&((x - preX) > (-1 * threshValue))){
							vy = 1;
						}
						if(((y - preY) > 5)&&((y - preY) < threshValue/2)){
							vx = 1;
						}
						if(((y - preY) < -5)&&((y - preY) > (-1 * threshValue/2))){
							vx = -1;
						}
						*/
					}else{
						//マルチタッチモード
						//昇降下降
						if(((x - preX) > 5)&&((x - preX) < threshValue)){
							vz = 1.0;
						}
						if(((x - preX) < -5)&&((x - preX) > (-1 * threshValue))){
							vz = -1.0;
						}
					}
				}
			}
				
			printf("cmode:%02d pX:%02d-%02d pY:%02d-%02d vx:%02.1f vy:%02.1f vz:%02.1f vr:%02.1f \n",ardrone.getCameraMode(), preX, x, preY, y, vx, vy, vz, vr);
			//ardrone.move3D(vx, vy, vz, vr);
			if(mArDroneCommandFlag == false)
			{
				MouseARMode = true;
				ardrone.move3D(vx,  vy, vz, vr);
				msleep(200);
				MouseARMode = false;
			}
			preX = x;
			preY = y;
		}
		//ardrone.move3D(0, 0, 0, 0);
	}

}

//setMouseCallbackへ渡すコールバック関数。
//この関数内でマウスからのイベント情報をMouseParamへ渡しています。
void mMouseEventfunc(int event, int x, int y, int flags, void  *param)
{
	//int MouseMode = 0;
	//int MouseCtrlMode = 0;

	MouseParam *mparam = (MouseParam*)param;
	//IplImage *img = (IplImage*)param;

	mparam->x = x;
	mparam->y = y;
	mparam->event = event;
	mparam->flags = flags;
    
	//CV_EVENT_FLAG_CTRLKEY   =8,
	switch (flags)
	{
		case CV_EVENT_FLAG_CTRLKEY:
			MouseCtrlMode = 1;
			//if(mMouseDebugPrint == true)
				printf("CV_EVENT_FLAG_CTRLKEY: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);

			goMousePoint( x, y, MouseCtrlMode);
			break;

		default:
			MouseCtrlMode = 0;
			//printf("CV_EVENT_FLAG_CTRLKEY: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);

	}

	switch (event)
	{
		case CV_EVENT_MOUSEMOVE:
			//printf("CV_EVENT_MOUSEMOVE: %d, %d\n", x, y);
			if((MouseMode != 0) && (MouseCtrlMode == 0))
			{
				//goMousePoint(x, y, MouseCtrlMode);
			}
			break;

		case CV_EVENT_LBUTTONDOWN:
			MouseMode = 1;
			goMousePoint(x, y, MouseCtrlMode);
			//MouseCtrlMode = 0;
			//if(mMouseDebugPrint == true)
				printf("CV_EVENT_LBUTTONDOWN: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;

		case CV_EVENT_LBUTTONUP:
			MouseMode = 0;
			MouseCtrlMode = 0;

			//if(mMouseDebugPrint == true)
				printf("CV_EVENT_LBUTTONUP: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			goMousePoint(0, 0, MouseCtrlMode);
			break;

		case CV_EVENT_RBUTTONDOWN:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_RBUTTONDOWN: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_MBUTTONDOWN:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_MBUTTONDOWN: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_RBUTTONUP:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_RBUTTONUP: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_MBUTTONUP:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_MBUTTONUP: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_LBUTTONDBLCLK:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_LBUTTONDBLCLK: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_RBUTTONDBLCLK:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_RBUTTONDBLCLK: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;
		case CV_EVENT_MBUTTONDBLCLK:
			if(mMouseDebugPrint == true)
				printf("CV_EVENT_MBUTTONDBLCLK: %d, %d, %d, %d\n", x, y, MouseMode, MouseCtrlMode);
			break;

		//printf("x:%02d y:%02d e:%02d F:%02d \n",x, y,event,flags);
	}
}



