/*******************************
*  @Function   get_depth_information
*  @Works      对两台相机进行单目标定，随后进行立体标定、校正、对应，生成具有深度信息的图像
*  @Author     Hunt Tiger Tonight
*  @Platform   VS2015 C++
*  @Connect    phone：18398621916/QQ:136768916
*  @Date       2018-10-29
********************************/

#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<string.h>

using namespace std;

void Camera_calibration(    int board_w,     //棋盘的宽度
							int board_h,     //棋盘的高度
							int n_boards,     //监测标定图像的数目，后面在输入参数里面获取，为了保证参数的求解精度，我们至少需要10张以上的图像      
							int delay,     //相机的拍摄延时为1s
							int image_sf,     //缩放比例为0.5
							int cap     //选择调用相机
)
{
	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);     //board_sz定义为size类型的数据

	//打开摄像头
	cv::VideoCapture capture(cap);
	if (!capture.isOpened())
	{
		cout << "\n无法打开摄像头。";
		return ;
	}
	//分配储存面
	vector<vector<cv::Point2f>> image_points;     //定义棋盘中图像角点的输出矩阵（向量中的向量）
	vector<vector<cv::Point3f>> object_points;     //定义物理坐标系中角点的输出矩阵（向量中的向量）

												   //相机不断拍摄图像，直到找到棋盘，并找齐所有的图像。
	double last_captured_timestamp = 0;     //初始化最后一次捕获图像时间为0
	cv::Size image_size;     //构造size型函数

							 //开始搜索，直到找到全部图像
	while (image_points.size() < (size_t)n_boards)
	{
		cv::Mat image0, image;     //构造原始图像矩阵以及输出图像矩阵
		capture >> image0;     //将原始图像存到capture中
		image_size = image0.size();     //获取image0d大小
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);     //缩放图像，函数解析详见P268

																						 //寻找棋盘
		vector<cv::Point2f> corners;     //定义角点输出矩阵
		bool found = cv::findChessboardCorners(image, board_sz, corners);     //寻找角点函数，详见p568

																			  //绘制棋盘
		drawChessboardCorners(image, board_sz, corners, found);     //绘制角点，详见p569

																	//如果找到棋盘了，就把他存入数据中
		double timestamp = (double)clock() / CLOCKS_PER_SEC;     //获取时间戳

		if (found && timestamp - last_captured_timestamp > 1)     //如果寻找到了棋盘
		{
			last_captured_timestamp = timestamp;     //将当前时间更新为最后一次时间
			image ^= cv::Scalar::all(255);     //将图像进行一次异或运算，255为白色，即：黑变白，白变黑
			cv::Mat mcorners(corners);     //复制矩阵（避免破坏原有矩阵）
			mcorners *= (1.0 / image_sf);     //缩放角坐标
			image_points.push_back(corners);     //在image_points后插入corners，这里注意一下，此举相当于，在image图像上叠加了一个棋盘图像
			object_points.push_back(vector<cv::Point3f>());     //在object_points后插入Point3f类型函数，同理，先加上一个还没有求解到的图像，用一个空矩阵表示
																//下面这段其实我觉得我的理解有点问题，我的理解是：将输出图像所占内存大小调整到最优，获取图像直到数目达到预设值
			vector<cv::Point3f> & opts = object_points.back();     //opts即：Options,简单来说就是将输出图像的最后一位大小最优
			opts.resize(board_n);     //调整容器大小
			for (int j = 0; j < board_n; j++)
			{
				opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.0f);     //将三维数据存入opts中,注意，这个地方必须加强制转换，不然会出错！！！！！！
			}
			cout << "已收集到" << static_cast<uint>(image_points.size()) << "张棋盘图像，总共需要" << n_boards << "张棋盘图像\n" << endl;
		}
		cv::imshow("Calibration", image);     //显示图像

		//等待时间为30ms，如果在这个时间段内, 用户按下ESC(ASCII码为27),则跳出循环,否则,则跳出循环
		if ((cv::waitKey(30)) & 255 == 27)
			return ;

	}
	//结束循环
	cv::destroyWindow("Calibration");     //销毁窗口
	cout << "\n\n***正在矫正相机...\n" << endl;

	//校准相机
	cv::Mat intrinsic_matrix, distortion_coeffs;     //instrinsic_matrix:线性内在参数，3*3矩阵， distortion_coeffs：畸变系数：k1、k2、p1、p2
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT

	);     //校准相机函数，详见P582

		   //保存相机内在参数以及畸变参数
	cout << "***Done!\n\nReprojection error is" << err << "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);     //保存文件为intrinsics.xml，开始写入文件
	fs << "image_width" << image_size.width << "image_height" << image_size.height << "camera_matrix" << intrinsic_matrix << "distortion_coefficients" << distortion_coeffs;
	fs.release();     //写入相机的宽和高，线性内在参数，畸变参数，然后释放缓存

					  //加载矩阵示例
	fs.open("intrinsics.xml", cv::FileStorage::READ);     //读取文件intrinsics.xml
	cout << "\nimage width:" << static_cast<int>(fs["image_width"]);     //读取图像宽，注意，这个地方必须加强制转换，不然会出错！！！！！！
	cout << "\nimage height:" << static_cast<int>(fs["image_height"]);     //读取图像高，注意，这个地方必须加强制转换，不然会出错！！！！！！

	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;     //创建矩阵相机参数读取矩阵，畸变系数读取矩阵
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix：" << intrinsic_matrix_loaded;
	cout << "\ndistortion matrix:" << distortion_coeffs_loaded << endl;     //读取并输出

																			//计算无畸变和修正转换映射
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
	);     //计算无畸变和修正转换映射，详见P590

		   //显示矫正的后的图像
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
		);     //利用remap重新传入图像
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