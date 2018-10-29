/*******************************
*  @Function   get_depth_information
*  @Works      ����̨������е�Ŀ�궨������������궨��У������Ӧ�����ɾ��������Ϣ��ͼ��
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone��18398621916/QQ:136768916
*  @Date       2018-10-29
********************************/

#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<string.h>

using namespace std;

void Camera_calibration(    int board_w,     //���̵Ŀ��
							int board_h,     //���̵ĸ߶�
							int n_boards,     //���궨ͼ�����Ŀ��������������������ȡ��Ϊ�˱�֤��������⾫�ȣ�����������Ҫ10�����ϵ�ͼ��      
							int delay,     //�����������ʱΪ1s
							int image_sf,     //���ű���Ϊ0.5
							int cap     //ѡ��������
)
{
	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);     //board_sz����Ϊsize���͵�����

	//������ͷ
	cv::VideoCapture capture(cap);
	if (!capture.isOpened())
	{
		cout << "\n�޷�������ͷ��";
		return ;
	}
	//���䴢����
	vector<vector<cv::Point2f>> image_points;     //����������ͼ��ǵ��������������е�������
	vector<vector<cv::Point3f>> object_points;     //������������ϵ�нǵ��������������е�������

												   //�����������ͼ��ֱ���ҵ����̣����������е�ͼ��
	double last_captured_timestamp = 0;     //��ʼ�����һ�β���ͼ��ʱ��Ϊ0
	cv::Size image_size;     //����size�ͺ���

							 //��ʼ������ֱ���ҵ�ȫ��ͼ��
	while (image_points.size() < (size_t)n_boards)
	{
		cv::Mat image0, image;     //����ԭʼͼ������Լ����ͼ�����
		capture >> image0;     //��ԭʼͼ��浽capture��
		image_size = image0.size();     //��ȡimage0d��С
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);     //����ͼ�񣬺����������P268

																						 //Ѱ������
		vector<cv::Point2f> corners;     //����ǵ��������
		bool found = cv::findChessboardCorners(image, board_sz, corners);     //Ѱ�ҽǵ㺯�������p568

																			  //��������
		drawChessboardCorners(image, board_sz, corners, found);     //���ƽǵ㣬���p569

																	//����ҵ������ˣ��Ͱ�������������
		double timestamp = (double)clock() / CLOCKS_PER_SEC;     //��ȡʱ���

		if (found && timestamp - last_captured_timestamp > 1)     //���Ѱ�ҵ�������
		{
			last_captured_timestamp = timestamp;     //����ǰʱ�����Ϊ���һ��ʱ��
			image ^= cv::Scalar::all(255);     //��ͼ�����һ��������㣬255Ϊ��ɫ�������ڱ�ף��ױ��
			cv::Mat mcorners(corners);     //���ƾ��󣨱����ƻ�ԭ�о���
			mcorners *= (1.0 / image_sf);     //���Ž�����
			image_points.push_back(corners);     //��image_points�����corners������ע��һ�£��˾��൱�ڣ���imageͼ���ϵ�����һ������ͼ��
			object_points.push_back(vector<cv::Point3f>());     //��object_points�����Point3f���ͺ�����ͬ���ȼ���һ����û����⵽��ͼ����һ���վ����ʾ
																//���������ʵ�Ҿ����ҵ�����е����⣬�ҵ�����ǣ������ͼ����ռ�ڴ��С���������ţ���ȡͼ��ֱ����Ŀ�ﵽԤ��ֵ
			vector<cv::Point3f> & opts = object_points.back();     //opts����Options,����˵���ǽ����ͼ������һλ��С����
			opts.resize(board_n);     //����������С
			for (int j = 0; j < board_n; j++)
			{
				opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.0f);     //����ά���ݴ���opts��,ע�⣬����ط������ǿ��ת������Ȼ���������������
			}
			cout << "���ռ���" << static_cast<uint>(image_points.size()) << "������ͼ���ܹ���Ҫ" << n_boards << "������ͼ��\n" << endl;
		}
		cv::imshow("Calibration", image);     //��ʾͼ��

		//�ȴ�ʱ��Ϊ30ms����������ʱ�����, �û�����ESC(ASCII��Ϊ27),������ѭ��,����,������ѭ��
		if ((cv::waitKey(30)) & 255 == 27)
			return ;

	}
	//����ѭ��
	cv::destroyWindow("Calibration");     //���ٴ���
	cout << "\n\n***���ڽ������...\n" << endl;

	//У׼���
	cv::Mat intrinsic_matrix, distortion_coeffs;     //instrinsic_matrix:�������ڲ�����3*3���� distortion_coeffs������ϵ����k1��k2��p1��p2
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT

	);     //У׼������������P582

		   //����������ڲ����Լ��������
	cout << "***Done!\n\nReprojection error is" << err << "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);     //�����ļ�Ϊintrinsics.xml����ʼд���ļ�
	fs << "image_width" << image_size.width << "image_height" << image_size.height << "camera_matrix" << intrinsic_matrix << "distortion_coefficients" << distortion_coeffs;
	fs.release();     //д������Ŀ�͸ߣ��������ڲ��������������Ȼ���ͷŻ���

					  //���ؾ���ʾ��
	fs.open("intrinsics.xml", cv::FileStorage::READ);     //��ȡ�ļ�intrinsics.xml
	cout << "\nimage width:" << static_cast<int>(fs["image_width"]);     //��ȡͼ���ע�⣬����ط������ǿ��ת������Ȼ���������������
	cout << "\nimage height:" << static_cast<int>(fs["image_height"]);     //��ȡͼ��ߣ�ע�⣬����ط������ǿ��ת������Ȼ���������������

	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;     //�����������������ȡ���󣬻���ϵ����ȡ����
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix��" << intrinsic_matrix_loaded;
	cout << "\ndistortion matrix:" << distortion_coeffs_loaded << endl;     //��ȡ�����

																			//�����޻��������ת��ӳ��
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix_loaded,
		distortion_coeffs_loaded,
		cv::Mat(),
		intrinsic_matrix_loaded,
		image_size,
		CV_16SC2,
		map1,
		map2
	);     //�����޻��������ת��ӳ�䣬���P590

		   //��ʾ�����ĺ��ͼ��
	for (;;)
	{
		cv::Mat image, image0;
		capture >> image0;
		if (image0.empty())
			break;
		cv::remap(
			image0,
			image,
			map1,
			map2,
			cv::INTER_LINEAR,
			cv::BORDER_CONSTANT,
			cv::Scalar()
		);     //����remap���´���ͼ��
		cv::imshow("Undistored", image);
		if ((cv::waitKey(30)) & 255 == 27)
			break;
	}
	return ;
}

int main(int argc, char *argv[])
{
	Camera_calibration(9,6,15,500,0.5,0);
}