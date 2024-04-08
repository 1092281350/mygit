#include <iostream>
#include <Eigen/Dense> // ��Ҫ��װ Eigen ��ѧ��
#include<vector>
#include <cmath>

using namespace std;
using namespace Eigen;
#define M_PI 3.1415926535
// ����Eigen�⣬����SVD�ֽ�ķ���������α�棬Ĭ�����erΪ0
//Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd& origin, const float er = 0) {
//	// ����svd�ֽ�
//	Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
//		Eigen::ComputeThinU |
//		Eigen::ComputeThinV);
//	// ����SVD�ֽ���
//	Eigen::MatrixXd U = svd_holder.matrixU();
//	Eigen::MatrixXd V = svd_holder.matrixV();
//	Eigen::MatrixXd D = svd_holder.singularValues();
//
//	// ����S����
//	Eigen::MatrixXd S(V.cols(), U.cols());
//	S.setZero();
//
//	for (unsigned int i = 0; i < D.size(); ++i) {
//
//		if (D(i, 0) > er) {
//			S(i, i) = 1 / D(i, 0);
//		}
//		else {
//			S(i, i) = 0;
//		}
//	}
//	return V * S * U.transpose();
//}


bool arePointsCollinear(const vector<Vector2d>& points) {
	MatrixXd A(points.size(), 2);
	VectorXd b(points.size());

	for (size_t i = 0; i < points.size(); ++i) {
		A(i, 0) = points[i](0);
		A(i, 1) = 1;
		b(i) = points[i](1);
	}

	VectorXd x = A.colPivHouseholderQr().solve(b);

	VectorXd residuals = b - A * x;
	double maxResidual = residuals.lpNorm<Infinity>();

	// ����һ�������ֵ�������������
	double threshold = 4e-2;

	return maxResidual <= threshold;
}


// ������������֮��ļнǣ����ȣ�
double calculateAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
	double dotProduct = p1.dot(p2);
	double normProduct = p1.norm() * p2.norm();
	return std::acos(dotProduct / normProduct);
}

// �����������γɵļнǣ����ȣ�
double calculateTriangleAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;
	return calculateAngle(v1, v2);
}

int main() {
	vector<Vector2d> points;
	points.push_back(Vector2d(0, 0));
	points.push_back(Vector2d(1, 1));
	points.push_back(Vector2d(1.95, 2.05));

	bool collinear = arePointsCollinear(points);

	if (collinear) {
		cout << "Points are collinear." << endl;
	}
	else {
		cout << "Points are not collinear." << endl;
	}

	Eigen::Vector3d p1(0.0, 1.0, 0.0);
	Eigen::Vector3d p2(0.0, 0.0, 0.0);
	Eigen::Vector3d p3(1.732, 0.0, 0.0);
	//���Ƕ���������������calculateAngle���ڼ�����������֮��ļнǣ�calculateTriangleAngle���ڼ����������γɵļнǡ�

	//	��calculateAngle�����У�����ʹ����Eigen���dot�������������������ĵ����ʹ��norm�������������ķ������������ĳ��ȣ���Ȼ��ͨ�����dotProduct / normProduct�ķ��������õ��нǵĻ���ֵ��

	//	��calculateTriangleAngle�����У��������ȼ�������������v1��v2�����Ƿֱ��ǵ�p2�͵�p3����ڵ�p1��������Ȼ�󣬵���calculateAngle��������������������֮��ļнǡ�

	double angle = calculateTriangleAngle(p1, p2, p3);

	std::cout << "�нǣ����ȣ�: " << angle << std::endl;
	std::cout << "�нǣ�������: " << angle * 180.0 / M_PI << std::endl;

	getchar();
	return 0;
}
//int main1() {
//
//	Eigen::MatrixXd B(7, 7);
//	B << 1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7,
//		1, 2, 3, 4, 5, 6, 7;
//
//	// ��ӡ����B��α�����
//	cout << B << endl;
//
//	cout << "����B��α��Ϊ��" << endl;
//
//	cout << pinv_eigen_based(B) << endl;
//
//	getchar();
//}
