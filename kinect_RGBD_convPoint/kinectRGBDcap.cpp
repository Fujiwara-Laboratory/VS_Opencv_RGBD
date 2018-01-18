#include <iostream>
#include <opencv2/opencv.hpp>

#include <Kinect.h>

#include <atlbase.h>

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

class KinectApp {
private:
	// Kinect SDK
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<ICoordinateMapper> coordinateMapper = nullptr;

	// RGB用の変数
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	std::vector<BYTE> colorBuffer;

	// D用の変数
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	std::vector<UINT16> depthBuffer;
public:
	int colorWidth;
	int colorHeight;

	int depthWidth;
	int depthHeight;

	~KinectApp() {
		// Kinectの動作を終了する
		if (kinect != nullptr) {
			kinect->Close();
		}
	}

	// 初期化
	void initialize() {
		// デフォルトのKinectを取得する
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());

		// 座標変換インタフェースを取得
		kinect->get_CoordinateMapper(&coordinateMapper);

		// カラーリーダーを取得する
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// RGB画像のサイズを取得する
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// RGB用のバッファーを作成する
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

		// Depthリーダーを取得する
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Depth画像のサイズを取得する
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		std::cout << "Depth横幅       : " << depthWidth << std::endl;
		std::cout << "Depth立幅       : " << depthHeight << std::endl;

		// Depthの最大値、最小値を取得する
		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepthReliableDistance));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepthReliableDistance));
		std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
		std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;

		// Depthのバッファーを作成する
		depthBuffer.resize(depthWidth * depthHeight);
	}

	// RGBDフレームの更新
	void updateRGBDFrame() {

		// RGBフレームを取得する
		CComPtr<IColorFrame> colorFrame;
		auto retRGB = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(retRGB)) return;

		// データを取得する
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], colorFormat));

		// Depthフレームを取得する
		CComPtr<IDepthFrame> depthFrame;
		auto retD = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (retD != S_OK) return;

		// データを取得する
		ERROR_CHECK(depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]));
	}

	// RGBをMat形式で取得
	void updateColorImage(cv::Mat &img) {
		img = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	}

	// RGBをDepthの空間に写像してMat形式で取得
	void updateColor2DepthImage(cv::Mat &img) {
		int i;
		// Depth座標系に対応するカラー座標系の一覧を取得する
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		for (i = 0; i < depthWidth * depthHeight; ++i) {
			// Depth座標系をベースにした、その点でRGBのどこを見るのかという座標を取得
			int colorX = (int)(colorSpace[i].X + 0.5); // 四捨五入
			int colorY = (int)(colorSpace[i].Y + 0.5);
			if ((colorX < 0) || (colorWidth <= colorX) || (colorY < 0) || (colorHeight <= colorY)) continue;

			int colorIndex = (colorY * colorWidth) + colorX;
			int colorImageIndex = i * colorBytesPerPixel;
			int colorBufferIndex = colorIndex * colorBytesPerPixel;
			img.data[colorImageIndex + 0] = colorBuffer[colorBufferIndex + 0];
			img.data[colorImageIndex + 1] = colorBuffer[colorBufferIndex + 1];
			img.data[colorImageIndex + 2] = colorBuffer[colorBufferIndex + 2];
		}
	}

	// RGB空間の座標をDepthの空間に写像
	void pointColor2DepthSpace(int x, int y, int &u, int &v) {
		// Depth座標系に対応するカラー座標系の一覧を取得する
		std::vector<DepthSpacePoint> depthSpacePoints(colorWidth * colorHeight);
		coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthSpacePoints.size(), &depthSpacePoints[0]);

		u = (int)(depthSpacePoints[x + y * colorWidth].X + 0.5); // 四捨五入
		v = (int)(depthSpacePoints[x + y * colorWidth].Y + 0.5); // 四捨五入
	}

	// DepthをRGBの空間に写像してMat形式で取得 + 256諧調に変換して取得 (最小値、最大値)
	void updateDepth2ColorCvtImage(cv::Mat &img, int min, int max) {
		int i, j, c = -1;
		float del = (max - min) / 255;
		UINT16 d;
		// Depth座標系に対応するカラー座標系の一覧を取得する
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		img = cv::Scalar(0); // 全てのピクセルがうまるわけではないので事前に初期化しておく
		for (i = 0; i < depthWidth; ++i) {
			for (j = 0; j < depthHeight; ++j) {
				c++;
				// RGB座標系をベースにした、その点でDepthのどこを見るのかという座標を取得
				int colorX = (int)(colorSpace[c].X + 0.5); // 四捨五入
				int colorY = (int)(colorSpace[c].Y + 0.5);
				if ((colorX < 0) || (colorWidth <= colorX) || (colorY < 0) || (colorHeight <= colorY)) continue;

				d = depthBuffer[c];
				if (d < min) img.at<uchar>(colorY, colorX) = 0;
				else if (d >= max) img.at<uchar>(colorY, colorX) = 255;
				else img.at<uchar>(colorY, colorX) = (uchar)((d - min) / del);
			}
		}
	}

	// DepthをRGBの空間に写像してMat形式で取得
	void updateDepth2ColorRawImage(cv::Mat &img) {
		int i, j, c = -1;
		// Depth座標系に対応するカラー座標系の一覧を取得する
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		img = cv::Scalar(0); // 全てのピクセルがうまるわけではないので事前に初期化しておく
		for (i = 0; i < depthWidth; ++i) {
			for (j = 0; j < depthHeight; ++j) {
				c++;
				// RGB座標系をベースにした、その点でDepthのどこを見るのかという座標を取得
				int colorX = (int)(colorSpace[c].X + 0.5); // 四捨五入
				int colorY = (int)(colorSpace[c].Y + 0.5);
				if ((colorX < 0) || (colorWidth <= colorX) || (colorY < 0) || (colorHeight <= colorY)) continue;

				img.at<UINT16>(colorY, colorX) = depthBuffer[c];
			}
		}
	}

	// DepthをMat形式の生データで取得
	void updateDepthRawImage(cv::Mat &img) {
		int i, j, c = 0;
		for (j = 0; j < depthHeight; j++) {
			for (i = 0; i < depthWidth; i++) {
				img.at<UINT16>(j, i) = depthBuffer[c];
				c++;
			}
		}
	}

	// DepthをMat形式の256諧調に変換して取得 (最小値、最大値)
	void updateDepthCvtImage(cv::Mat &img, int min, int max) {
		int i, j, c = 0;
		float del = (max - min) / 255;
		UINT16 d;
		for (j = 0; j < depthHeight; j++) {
			for (i = 0; i < depthWidth; i++) {
				d = depthBuffer[c];
				if (d < min) img.at<uchar>(j, i) = 0;
				else if (d >= max) img.at<uchar>(j, i) = 255;
				else img.at<uchar>(j, i) = (uchar)((d - min) / del);
				c++;
			}
		}
	}
};

// マウス関連の処理
int mouseX, mouseY, mouseW, mouseH, btnFlag;
cv::Mat dispDepColM;
void onMouse(int event, int x, int y, int flags, void*) {
	if (x < 0 || y < 0 || x > mouseW || y > mouseH) return;

	if((event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_MOUSEMOVE) && (flags & CV_EVENT_FLAG_LBUTTON)){
		btnFlag = 1;
		mouseX = x;
		mouseY = y;
	}else{
		btnFlag = -1;
	}
}

int main() {
	KinectApp knct;
	cv::Mat FHDrgbM, depRawM, dispColM, dispDepM;
	cv::Mat depRGBspM, FHDrgbDspRawM, FHDrgbDspM, rgbDspM;

	try { knct.initialize(); } // Kinectの初期化
	catch (std::exception& ex) { std::cout << ex.what() << std::endl; }

	depRawM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_16UC1);
	dispDepM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_8UC1);
	depRGBspM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_8UC4);
	FHDrgbDspRawM = cv::Mat(knct.colorHeight, knct.colorWidth, CV_16UC1);
	FHDrgbDspM = cv::Mat(knct.colorHeight, knct.colorWidth, CV_8UC1);

	float resizeScale = 0.5;
	int convX, convY;
	mouseW = knct.colorWidth * resizeScale;
	mouseH = knct.colorHeight * resizeScale;
	btnFlag = -1;

	cv::namedWindow("color Image", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
	// マウスイベントに対するコールバック関数を登録
	cv::setMouseCallback("color Image", onMouse, 0);

	while (1) { // メインループ
		knct.updateRGBDFrame();


		// RGB画像の取得
		knct.updateColorImage(FHDrgbM);
		cv::resize(FHDrgbM, dispColM, cv::Size(), resizeScale, resizeScale); // 表示用にリサイズ

		cv::imshow("color Image", dispColM); // RGB画像の表示

		// 距離画像の生データ(2byteデータ)での取得
		//float min = 600, max = 1000, del = (max - min) / 255;
		//knct.updateDepthRawImage(depRawM); // データの取得
		//for (int j = 0; j < knct.depthHeight; j++) {
		//	for (int i = 0; i < knct.depthWidth; i++) {
		//		UINT16 d = depRawM.at<UINT16>(j, i);
		//		if (d < min) dispDepM.at<uchar>(j, i) = 0;
		//		else if(d >= max) dispDepM.at<uchar>(j, i) = 255;
		//		else dispDepM.at<uchar>(j, i) = (uchar)((d - min) / del);
		//	}
		//}

		// 諧調補正をした距離画像の取得 (最小値-最大値間を256諧調に)
		knct.updateDepthCvtImage(dispDepM, 600, 3000);

		if (btnFlag == 1) {
			cv::cvtColor(dispDepM, dispDepColM, CV_GRAY2BGR);
			knct.pointColor2DepthSpace(mouseX * 2, mouseY * 2, convX, convY);
			cv::circle(dispDepColM, cv::Point(convX, convY), 3, cv::Scalar(0, 0, 255), 1, CV_AA);

			cv::imshow("depth Image", dispDepColM); // 距離画像の表示
		} else {
			cv::imshow("depth Image", dispDepM); // 距離画像の表示
		}

		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
	return 0;
}

