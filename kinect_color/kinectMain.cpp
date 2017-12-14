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

	// �\������
	std::vector<BYTE> colorBuffer;

public:
	~KinectApp(){
		// Kinect�̓�����I������
		if (kinect != nullptr) {
			kinect->Close();
		}
	}

	// ������
	void initialize(){
		// �f�t�H���g��Kinect���擾����
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());

		// �J���[���[�_�[���擾����
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// �f�t�H���g�̃J���[�摜�̃T�C�Y���擾����
		CComPtr<IFrameDescription> defaultColorFrameDescription;
		ERROR_CHECK(colorFrameSource->get_FrameDescription(&defaultColorFrameDescription));
		ERROR_CHECK(defaultColorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(defaultColorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(defaultColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "default : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// �J���[�摜�̃T�C�Y���擾����
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// �o�b�t�@�[���쐬����
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	// �J���[�t���[���̍X�V
	void updateColorFrame(){
		// �t���[�����擾����
		CComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret))return;

		// �w��̌`���Ńf�[�^���擾����
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], colorFormat));
	}

	// RGB��Mat�`���Ŏ擾
	void updateColorImage(cv::Mat &img) {
		img = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	}
};

int main() {
	KinectApp knct;
	cv::Mat capM, dispM;

	try { knct.initialize(); } // Kinect�̏�����
	catch (std::exception& ex) { std::cout << ex.what() << std::endl; }

	while (1) { // ���C�����[�v
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