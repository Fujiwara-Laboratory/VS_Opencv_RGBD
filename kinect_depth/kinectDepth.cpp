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
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;

	std::vector<UINT16> depthBuffer;
public:
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

		// Depthリーダーを取得する
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Depth画像のサイズを取得する
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription( &depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		std::cout << "Depth横幅       : " << depthWidth << std::endl;
		std::cout << "Depth立幅       : " << depthHeight << std::endl;
		
		// Depthの最大値、最小値を取得する
		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance));
		std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
		std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;

		// バッファーを作成する
		depthBuffer.resize(depthWidth * depthHeight);
	}

	// Depthフレームの更新
	void updateDepthFrame(){
		// Depthフレームを取得する
		CComPtr<IDepthFrame> depthFrame;
		auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (ret != S_OK) return;

		// データを取得する
		ERROR_CHECK(depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0]));
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

int main() {
	KinectApp knct;
	cv::Mat depRawM, dispM;

	try { knct.initialize(); } // Kinectの初期化
	catch (std::exception& ex) { std::cout << ex.what() << std::endl; }

	depRawM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_16UC1);
	dispM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_8UC1);

	while (1) { // メインループ
		knct.updateDepthFrame();

		//生データから変換する場合
		/*
		int i, j;
		float min, max, del;
		min = 600;
		max = 1000;
		del = (max - min) / 255;
		knct.updateDepthRawImage(depRawM);
		for (j = 0; j < knct.depthHeight; j++) {
			for (i = 0; i < knct.depthWidth; i++) {
				UINT16 d = depRawM.at<UINT16>(j, i);
				if (d < min) dispM.at<uchar>(j, i) = 0;
				else if(d >= max) dispM.at<uchar>(j, i) = 255;
				else dispM.at<uchar>(j, i) = (uchar)((d - min) / del);
			}
		}
		*/

		knct.updateDepthCvtImage(dispM, 600, 3000);

		cv::imshow("depth Image", dispM);
		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
	return 0;
}