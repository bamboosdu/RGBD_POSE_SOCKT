// #include <iostream>
// #include <fstream>
// #include <assert.h>
// #include <string>

// //// pcl
// //#include <pcl/io/pcd_io.h>
// //#include <pcl/point_types.h>
// //#include <pcl/registration/gicp6d.h>

// // OpenCV
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// // Eigen
// #include <Eigen/Dense> 
// #include <Eigen/src/Geometry/Quaternion.h>
// // octomap 
// #include <octomap/octomap.h>
// #include <octomap/ColorOcTree.h>
// using namespace std;

// //// internal 
// //const double camera_factor = 1000;
// //const double camera_cx = 319.5;
// //const double camera_cy = 239.5;
// //const double camera_fx = 533.9;
// //const double camera_fy = 533.9;
// // internal 
// const double camera_factor = 1000;
// const double camera_cx = 320.5;
// const double camera_cy = 240.5;
// const double camera_fx = 554.38;
// const double camera_fy = 554.38;
// // img
// cv::Mat depth;
// // 
// string outdir;

// const float cellsize = 0.05;
// int scal = 3;



// int main(){
// 	//depth = cv::imread("D:/works/TestOc/TestOc/TestOc/data/depth_test.png", -1); //不修改原图???
// 	//outdir = "D:/works/TestOc/TestOc/TestOc/data/depth_test.bt";
// 	//writeOctomap(depth, outdir);
// 	ifstream ifs;
// 	ifs.open("src/virtual_scan/data/pose.txt");
// printf("1\n");
// 	// pc octo
// 	octomap::Pointcloud* pc = new octomap::Pointcloud();

// 	/*// test
// 	depth = cv::imread("D:/works/TestOc/TestOc/TestOc/data/depth3.png");
// 	float pose[7];
// 	for (int i = 0; i < 4; i++)
// 		ifs >> pose[0] >> pose[1] >> pose[2] >> pose[3] >> pose[4] >> pose[5] >> pose[6];
// 	for (int m = 0; m < depth.rows; m++){
// 		for (int n = 0; n < depth.cols; n++){
// 			// (m,n) d
// 			ushort d = depth.ptr<ushort>(m)[n];
// 			if (d == 0)
// 				continue;
// 			// 3d coordinate
// 			float z = double(d) / camera_factor;
// 			float x = (n - camera_cx) * z / camera_fx;
// 			float y = (m - camera_cy) * z / camera_fy;

// 			Eigen::Quaternion<float> q(pose[6], pose[3], pose[4], pose[5]);
// 			Eigen::MatrixXf r = q.toRotationMatrix();
// 			// 转到gazebo坐标系
// 			Eigen::Vector3f p;
// 			p[0] = z, p[1] = -x, p[2] = y;
// 			Eigen::Vector3f t;
// 			t[0] = pose[0], t[1] = pose[1], t[2] = pose[2];
// 			p = r*p + t;
// 			// 转回坐标系
// 			pc->push_back(-p[1], p[2], p[0]);
// 			// add p to octomap
// 			//pc->push_back(p[0], p[1], p[2]);
// 			//cout << "(x, y, z): " << p[0] << ", " << p[1] << ", " << p[2] << endl;
// 		}
// 	}
// 	// test */
// printf("2\n");
// 	int num = 9;
// 	for (int i = 0; i < num; i++)
// 	{
// 		stringstream ss;
// 		string s;
// 		ss << "src/virtual_scan/data/depth" << i << ".png";
// 		ss >> s;
// 		depth = cv::imread(s, -1);
// 		float pose[7];
// 		ifs >> pose[0] >> pose[1] >> pose[2] >> pose[3] >> pose[4] >> pose[5] >> pose[6];
// 		//cout << pose[0] << ", " << pose[1] << ", " << pose[2] << ", " << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << ", " << endl;
// 		for (int m = 0; m < depth.rows; m++){
// 			for (int n = 0; n < depth.cols; n++){
// 				// (m,n) d
// 				ushort d = depth.ptr<ushort>(m)[n];
// 				if (d == 0)
// 					continue;
// 				// 3d coordinate
// 				float z = double(d) / camera_factor;
// 				float x = (n - camera_cx) * z / camera_fx;
// 				float y = (m - camera_cy) * z / camera_fy;
// 				Eigen::Quaternion<float> q(pose[6], pose[3], pose[4], pose[5]);
// 				Eigen::MatrixXf r = q.toRotationMatrix();
// 				// 转到gazebo坐标系
// 				Eigen::Vector3f p;
// 				p[0] = z, p[1] = -x, p[2] = y;
// 				Eigen::Vector3f t;
// 				t[0] = pose[0], t[1] = pose[1], t[2] = pose[2];
// 				p = r*p + t;
// 				// 转回坐标系
// 				pc->push_back(-p[1], p[2], p[0]);
// 			}
// 		}
// 	}
// 	ifs.close();//*/
// printf("3\n");
// 	octomap::OcTree* tree = new octomap::OcTree(cellsize);
// 	tree->insertPointCloudRays(pc, octomap::point3d(0, 0, 0));
// 	// octomap
// 	tree->updateInnerOccupancy();// what???
// printf("4\n");

// 	// to map
// 	float xmax = 0, xmin = 0, zmax = 0, zmin = 0;
// 	for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it){
// 		if (it.getX() < xmin)
// 			xmin = it.getX();
// 		if (it.getX() > xmax)
// 			xmax = it.getX();
// 		if (it.getZ() < zmin)
// 			zmin = it.getZ();
// 		if (it.getZ() > zmax)
// 			zmax = it.getZ();
// 	}

// 	int col = (int)((xmax - xmin + 1) / cellsize)*scal;
// 	int row = (int)((zmax - zmin + 1) / cellsize)*scal;
// 	cout << "row: " << row << ", col: " << col << endl;
// 	cv::Mat map(row, col, CV_8UC3);
// 	cv::imshow("empty", map);
// 	cv::waitKey(0);
// 	for (int i = 0; i < row; i++)
// 	{
// 		for (int j = 0; j < col; j++)
// 		{
// 			map.ptr<cv::Vec3b>(i)[j][0] = 0;
// 			map.ptr<cv::Vec3b>(i)[j][1] = 0;
// 			map.ptr<cv::Vec3b>(i)[j][2] = 0;
// 		}
// 	}
// 	//cv::imshow("empty", map);
// 	//cv::waitKey(0);
// 	int count = 0;
// 	for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it){
// 		//// cout info
// 		//std::cout << "Node depth: " << it.getDepth() << std::endl;
// 		//std::cout << "Node center: " << it.getCoordinate() << std::endl;
// 		//std::cout << "Node size: " << it.getSize() << std::endl;
// 		//std::cout << "Node value: " << it->getValue() << std::endl;
// 		//std::cout << "Node occupancy: " << tree->search(it.getKey())->getOccupancy() << std::endl << std::endl;
// 		int v = (int)((it.getX() - xmin) / cellsize)*scal;
// 		int u = (int)((zmax - it.getZ()) / cellsize)*scal;
// 		if (u == row - 1 && v == col - 1){
// 			//map.ptr<cv::Vec3b>(u)[v][0] = 255;
// 			//map.ptr<cv::Vec3b>(u)[v][1] = 255;
// 			//map.ptr<cv::Vec3b>(u)[v][2] = 255;
// 			if (tree->search(it.getKey())->getOccupancy() > 0.5){
// 				map.ptr<cv::Vec3b>(u)[v][0] = 0;
// 				map.ptr<cv::Vec3b>(u)[v][1] = 0;
// 				map.ptr<cv::Vec3b>(u)[v][2] = 255;
// 			}
// 		}
// 		else if (u == row - 1 && v != col - 1){
// 			for (int i = 0; i < scal; i++)
// 			{
// 				//map.ptr<cv::Vec3b>(u)[v + i][0] = 255;
// 				//map.ptr<cv::Vec3b>(u)[v + i][1] = 255;
// 				//map.ptr<cv::Vec3b>(u)[v + i][2] = 255;
// 				if (tree->search(it.getKey())->getOccupancy() > 0.5){
// 					map.ptr<cv::Vec3b>(u)[v+i][0] = 0;
// 					map.ptr<cv::Vec3b>(u)[v+i][1] = 0;
// 					map.ptr<cv::Vec3b>(u)[v + i][2] = 255;
// 				}
// 			}
// 		}
// 		else if (u != row - 1 && v == col - 1){
// 			for (int i = 0; i < scal; i++)
// 			{
// 				//map.ptr<cv::Vec3b>(u + i)[v][0] = 255;
// 				//map.ptr<cv::Vec3b>(u + i)[v][1] = 255;
// 				//map.ptr<cv::Vec3b>(u + i)[v][2] = 255;
// 				if (tree->search(it.getKey())->getOccupancy() > 0.5){
// 					map.ptr<cv::Vec3b>(u+i)[v][0] = 0;
// 					map.ptr<cv::Vec3b>(u+i)[v][1] = 0;
// 					map.ptr<cv::Vec3b>(u + i)[v][2] = 255;
// 				}
// 			}
// 		}
// 		else{
// 			for (int i = 0; i < scal; i++)
// 			{
// 				for (int j = 0; j < scal; j++)
// 				{
// 					//map.ptr<cv::Vec3b>(u + i)[v + j][0] = 255;
// 					//map.ptr<cv::Vec3b>(u + i)[v + j][1] = 255;
// 					//map.ptr<cv::Vec3b>(u + i)[v + j][2] = 255;
// 					if (tree->search(it.getKey())->getOccupancy() > 0.5){
// 						map.ptr<cv::Vec3b>(u + i)[v+j][0] = 0;
// 						map.ptr<cv::Vec3b>(u + i)[v+j][1] = 0;
// 						map.ptr<cv::Vec3b>(u + i)[v + j][2] = 255;
// 					}
// 				}
// 			}
// 		}
// 		/*
// 		//cout << "u: " << u << ", v: " << v << endl;
// 		map.ptr<cv::Vec3b>(u)[v][1] = 255;
// 		if (tree->search(it.getKey())->getOccupancy() > 0.5){
// 			map.ptr<cv::Vec3b>(u)[v][0] = 255;
// 			map.ptr<cv::Vec3b>(u)[v][1] = 255;
// 			map.ptr<cv::Vec3b>(u)[v][2] = 255;
// 		}//*/
// 		//cout << "image coor: " << u << ", " << v << endl;
// 		count++;
// 	}//*/
// 	cout << "num of nodes: " << count << endl;
// 	cv::imshow("map", map);
// 	cv::waitKey(0);

// 	// write
// 	stringstream ss;
// 	string s;
// 	ss << "src/virtual_scan/data/map" << num << ".png";
// 	ss >> s;
// 	cv::imwrite(s, map);

// 	/*// write octomap
// 	cout << "write data into .bt file..." << endl;
// 	outdir = "D:/works/TestOc/TestOc/TestOc/data/depth_fuse.bt";
// 	tree->writeBinary(outdir);
// 	cout << "done." << endl;//*/
	
// 	delete tree;
// 	delete pc;

// 	getchar();
// 	return 0;
// }