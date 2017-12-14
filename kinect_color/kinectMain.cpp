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

class KinectApp{
private:
	// Kinect SDK
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;

	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;

	// 表示部分
	std::vector<BYTE> colorBuffer;

public:
	~KinectApp(){
		// Kinectの動作を終了する
		if (kinect != nullptr) {
			kinect->Close();
		}
	}

	// 初期化
	void initialize(){
		// デフォルトのKinectを取得する
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());

		// カラーリーダーを取得する
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// デフォルトのカラー画像のサイズを取得する
		CComPtr<IFrameDescription> defaultColorFrameDescription;
		ERROR_CHECK(colorFrameSource->get_FrameDescription(&defaultColorFrameDescription));
		ERROR_CHECK(defaultColorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(defaultColorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(defaultColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "default : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// カラー画像のサイズを取得する
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// バッファーを作成する
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	// カラーフレームの更新
	void updateColorFrame(){
		// フレームを取得する
		CComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret))return;

		// 指定の形式でデータを取得する
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], colorFormat));
	}

	// RGBをMat形式で取得
	void updateColorImage(cv::Mat &img) {
		img = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	}
};

int main() {
	KinectApp knct;
	cv::Mat capM, dispM;

	try { knct.initialize(); } // Kinectの初期化
	catch (std::exception& ex) { std::cout << ex.what() << std::endl; }

	while (1) { // メインループ
		knct.updateColorFrame();
		knct.updateColorImage(capM);
		cv::resize(capM, dispM, cv::Size(), 0.5, 0.5);

		cv::imshow("color Image", dispM);
		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
	return 0;
}