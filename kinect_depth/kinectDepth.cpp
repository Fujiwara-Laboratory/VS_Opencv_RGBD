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
		// Kinect�̓�����I������
		if (kinect != nullptr) {
			kinect->Close();
		}
	}

	// ������
	void initialize() {
		// �f�t�H���g��Kinect���擾����
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());

		// Depth���[�_�[���擾����
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Depth�摜�̃T�C�Y���擾����
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription( &depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		std::cout << "Depth����       : " << depthWidth << std::endl;
		std::cout << "Depth����       : " << depthHeight << std::endl;
		
		// Depth�̍ő�l�A�ŏ��l���擾����
		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance));
		std::cout << "Depth�ŏ��l       : " << minDepthReliableDistance << std::endl;
		std::cout << "Depth�ő�l       : " << maxDepthReliableDistance << std::endl;

		// �o�b�t�@�[���쐬����
		depthBuffer.resize(depthWidth * depthHeight);
	}

	// Depth�t���[���̍X�V
	void updateDepthFrame(){
		// Depth�t���[�����擾����
		CComPtr<IDepthFrame> depthFrame;
		auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (ret != S_OK) return;

		// �f�[�^���擾����
		ERROR_CHECK(depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0]));
	}

	// Depth��Mat�`���̐��f�[�^�Ŏ擾
	void updateDepthRawImage(cv::Mat &img) {
		int i, j, c = 0;
		for (j = 0; j < depthHeight; j++) {
			for (i = 0; i < depthWidth; i++) {
				img.at<UINT16>(j, i) = depthBuffer[c];
				c++;
			}
		}
	}

	// Depth��Mat�`����256�~���ɕϊ����Ď擾 (�ŏ��l�A�ő�l)
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

	try { knct.initialize(); } // Kinect�̏�����
	catch (std::exception& ex) { std::cout << ex.what() << std::endl; }

	depRawM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_16UC1);
	dispM = cv::Mat(knct.depthHeight, knct.depthWidth, CV_8UC1);

	while (1) { // ���C�����[�v
		knct.updateDepthFrame();

		//���f�[�^����ϊ�����ꍇ
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