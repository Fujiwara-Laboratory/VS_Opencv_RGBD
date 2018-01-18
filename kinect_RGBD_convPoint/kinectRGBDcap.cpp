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

	// RGB�p�̕ϐ�
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	std::vector<BYTE> colorBuffer;

	// D�p�̕ϐ�
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
	std::vector<UINT16> depthBuffer;
public:
	int colorWidth;
	int colorHeight;

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

		// ���W�ϊ��C���^�t�F�[�X���擾
		kinect->get_CoordinateMapper(&coordinateMapper);

		// �J���[���[�_�[���擾����
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// RGB�摜�̃T�C�Y���擾����
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// RGB�p�̃o�b�t�@�[���쐬����
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

		// Depth���[�_�[���擾����
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Depth�摜�̃T�C�Y���擾����
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		std::cout << "Depth����       : " << depthWidth << std::endl;
		std::cout << "Depth����       : " << depthHeight << std::endl;

		// Depth�̍ő�l�A�ŏ��l���擾����
		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepthReliableDistance));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepthReliableDistance));
		std::cout << "Depth�ŏ��l       : " << minDepthReliableDistance << std::endl;
		std::cout << "Depth�ő�l       : " << maxDepthReliableDistance << std::endl;

		// Depth�̃o�b�t�@�[���쐬����
		depthBuffer.resize(depthWidth * depthHeight);
	}

	// RGBD�t���[���̍X�V
	void updateRGBDFrame() {

		// RGB�t���[�����擾����
		CComPtr<IColorFrame> colorFrame;
		auto retRGB = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(retRGB)) return;

		// �f�[�^���擾����
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], colorFormat));

		// Depth�t���[�����擾����
		CComPtr<IDepthFrame> depthFrame;
		auto retD = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (retD != S_OK) return;

		// �f�[�^���擾����
		ERROR_CHECK(depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]));
	}

	// RGB��Mat�`���Ŏ擾
	void updateColorImage(cv::Mat &img) {
		img = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	}

	// RGB��Depth�̋�ԂɎʑ�����Mat�`���Ŏ擾
	void updateColor2DepthImage(cv::Mat &img) {
		int i;
		// Depth���W�n�ɑΉ�����J���[���W�n�̈ꗗ���擾����
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		for (i = 0; i < depthWidth * depthHeight; ++i) {
			// Depth���W�n���x�[�X�ɂ����A���̓_��RGB�̂ǂ�������̂��Ƃ������W���擾
			int colorX = (int)(colorSpace[i].X + 0.5); // �l�̌ܓ�
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

	// RGB��Ԃ̍��W��Depth�̋�ԂɎʑ�
	void pointColor2DepthSpace(int x, int y, int &u, int &v) {
		// Depth���W�n�ɑΉ�����J���[���W�n�̈ꗗ���擾����
		std::vector<DepthSpacePoint> depthSpacePoints(colorWidth * colorHeight);
		coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthSpacePoints.size(), &depthSpacePoints[0]);

		u = (int)(depthSpacePoints[x + y * colorWidth].X + 0.5); // �l�̌ܓ�
		v = (int)(depthSpacePoints[x + y * colorWidth].Y + 0.5); // �l�̌ܓ�
	}

	// Depth��RGB�̋�ԂɎʑ�����Mat�`���Ŏ擾 + 256�~���ɕϊ����Ď擾 (�ŏ��l�A�ő�l)
	void updateDepth2ColorCvtImage(cv::Mat &img, int min, int max) {
		int i, j, c = -1;
		float del = (max - min) / 255;
		UINT16 d;
		// Depth���W�n�ɑΉ�����J���[���W�n�̈ꗗ���擾����
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		img = cv::Scalar(0); // �S�Ẵs�N�Z�������܂�킯�ł͂Ȃ��̂Ŏ��O�ɏ��������Ă���
		for (i = 0; i < depthWidth; ++i) {
			for (j = 0; j < depthHeight; ++j) {
				c++;
				// RGB���W�n���x�[�X�ɂ����A���̓_��Depth�̂ǂ�������̂��Ƃ������W���擾
				int colorX = (int)(colorSpace[c].X + 0.5); // �l�̌ܓ�
				int colorY = (int)(colorSpace[c].Y + 0.5);
				if ((colorX < 0) || (colorWidth <= colorX) || (colorY < 0) || (colorHeight <= colorY)) continue;

				d = depthBuffer[c];
				if (d < min) img.at<uchar>(colorY, colorX) = 0;
				else if (d >= max) img.at<uchar>(colorY, colorX) = 255;
				else img.at<uchar>(colorY, colorX) = (uchar)((d - min) / del);
			}
		}
	}

	// Depth��RGB�̋�ԂɎʑ�����Mat�`���Ŏ擾
	void updateDepth2ColorRawImage(cv::Mat &img) {
		int i, j, c = -1;
		// Depth���W�n�ɑΉ�����J���[���W�n�̈ꗗ���擾����
		std::vector<ColorSpacePoint> colorSpace(depthWidth * depthHeight);
		coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0]);

		img = cv::Scalar(0); // �S�Ẵs�N�Z�������܂�킯�ł͂Ȃ��̂Ŏ��O�ɏ��������Ă���
		for (i = 0; i < depthWidth; ++i) {
			for (j = 0; j < depthHeight; ++j) {
				c++;
				// RGB���W�n���x�[�X�ɂ����A���̓_��Depth�̂ǂ�������̂��Ƃ������W���擾
				int colorX = (int)(colorSpace[c].X + 0.5); // �l�̌ܓ�
				int colorY = (int)(colorSpace[c].Y + 0.5);
				if ((colorX < 0) || (colorWidth <= colorX) || (colorY < 0) || (colorHeight <= colorY)) continue;

				img.at<UINT16>(colorY, colorX) = depthBuffer[c];
			}
		}
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

// �}�E�X�֘A�̏���
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

	try { knct.initialize(); } // Kinect�̏�����
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
	// �}�E�X�C�x���g�ɑ΂���R�[���o�b�N�֐���o�^
	cv::setMouseCallback("color Image", onMouse, 0);

	while (1) { // ���C�����[�v
		knct.updateRGBDFrame();


		// RGB�摜�̎擾
		knct.updateColorImage(FHDrgbM);
		cv::resize(FHDrgbM, dispColM, cv::Size(), resizeScale, resizeScale); // �\���p�Ƀ��T�C�Y

		cv::imshow("color Image", dispColM); // RGB�摜�̕\��

		// �����摜�̐��f�[�^(2byte�f�[�^)�ł̎擾
		//float min = 600, max = 1000, del = (max - min) / 255;
		//knct.updateDepthRawImage(depRawM); // �f�[�^�̎擾
		//for (int j = 0; j < knct.depthHeight; j++) {
		//	for (int i = 0; i < knct.depthWidth; i++) {
		//		UINT16 d = depRawM.at<UINT16>(j, i);
		//		if (d < min) dispDepM.at<uchar>(j, i) = 0;
		//		else if(d >= max) dispDepM.at<uchar>(j, i) = 255;
		//		else dispDepM.at<uchar>(j, i) = (uchar)((d - min) / del);
		//	}
		//}

		// �~���␳�����������摜�̎擾 (�ŏ��l-�ő�l�Ԃ�256�~����)
		knct.updateDepthCvtImage(dispDepM, 600, 3000);

		if (btnFlag == 1) {
			cv::cvtColor(dispDepM, dispDepColM, CV_GRAY2BGR);
			knct.pointColor2DepthSpace(mouseX * 2, mouseY * 2, convX, convY);
			cv::circle(dispDepColM, cv::Point(convX, convY), 3, cv::Scalar(0, 0, 255), 1, CV_AA);

			cv::imshow("depth Image", dispDepColM); // �����摜�̕\��
		} else {
			cv::imshow("depth Image", dispDepM); // �����摜�̕\��
		}

		auto key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
	return 0;
}

