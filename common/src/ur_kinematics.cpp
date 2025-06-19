#include "ur_kinematics.hpp"

ur_frame *ur_frame::singleton =nullptr;

void ur_frame::init(const UrParam &ur_dh){
    if(singleton != nullptr){
        delete singleton;
    }
    singleton = new ur_frame(ur_dh);
}

ur_frame* ur_frame::instance()
{
    return singleton;
}

ur_frame::ur_frame(const UrParam &ur_dh){
    param.L1 = ur_dh.L1;
    param.L2 = ur_dh.L2;
    param.W1 = ur_dh.W1;
    param.W2 = ur_dh.W2;
    param.H1 = ur_dh.H1;
    param.H2 = ur_dh.H2;
}

ur_frame::~ur_frame(){
    singleton =nullptr;
}


Eigen::MatrixXd ur_frame::forward_kinematics(const mjModel* m,mjData* d){
    //采用《现代机器人学》书中旋量法对位姿进行正解

    double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
    //初始位姿
    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, param.L1+ param.L2,
        0, 0, 1, param.W1+param.W2, 
        0, 1, 0, param.H1-param.H2,
        0, 0, 0, 1;
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 0, 1,
        1, 0, 0, 0, -1, 0,
        0, -param.H1, -param.H1, -param.H1, -param.W1, param.H2 - param.H1,
        0, 0, 0, 0, param.L1 + param.L2, 0,
        0, 0, param.L1, param.L1 + param.L2, 0, param.L1 + param.L2;
    //std::cout << "Slist :" << Slist << std::endl;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];
    // std::cout<<"act_q"<<thetaList.transpose()<<std::endl;
    Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

    // std::cout << "FKCals :" << std::endl;
    // std::cout << FKCal << std::endl;
    // std::cout  << std::endl; 
    
    // std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
    return FKCal;
}


// Eigen::MatrixXd ur5_FK(const mjModel* m,mjData* d){
//     //采用《现代机器人学》书中旋量法对位姿进行正解

//     double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
//     //ur5
//     double param.L1 = 0.425;  
//     double param.L2 = 0.39225;
//     double param.W1 = 0.10915; 
//     double param.W2 = 0.0823;  
//     double param.H1 = 0.089159;
//     double param.H2 = 0.09465; 
//     //初始位姿
//     Eigen::MatrixXd M(4, 4);
//     M << -1, 0, 0, param.L1+ param.L2,
//         0, 0, 1, param.W1+param.W2,
//         0, 1, 0, param.H1-param.H2,
//         0, 0, 0, 1;
//     Eigen::MatrixXd Slist(6, 6);
//     //s矩阵
//     //列排列，非行排列
//     Slist << 0, 0,    0,    0,		0,		0,
//                 0, 1,    1,    1,		0,		1,
//                 1, 0,    0,	0,		-1,		0,
//                 0, -param.H1, -param.H1,  -param.H1,		-param.W1,	param.H2 - param.H1,
//                 0, 0,    0,	0,		param.L1 + param.L2,0,
//                 0, 0,    param.L1,	param.L1 + param.L2, 0,		 param.L1 + param.L2;
//     //std::cout << "Slist :" << Slist << std::endl;

//     Eigen::VectorXd thetaList(6);
//     thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
//     d->qpos[3], d->qpos[4], d->qpos[5];
//     Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

//     std::cout << "FKCals :" << std::endl;
//     std::cout << FKCal << std::endl;
//     std::cout  << std::endl; 
    
//     std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
//     return FKCal;
// }


Eigen::MatrixXd ur_frame::forward_kinematics(const mjModel* m,mjData* d, double *qpos){
    //采用《现代机器人学》书中旋量法对位姿进行正解

    double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
    //初始位姿
    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, param.L1+ param.L2,
        0, 0, 1, param.W1+param.W2,
        0, 1, 0, param.H1-param.H2,
        0, 0, 0, 1;
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -param.H1, -param.H1,  -param.H1,		-param.W1,	param.H2 - param.H1,
                0, 0,    0,	0,		param.L1 + param.L2,0,
                0, 0,    param.L1,	param.L1 + param.L2, 0,		 param.L1 + param.L2;
    //std::cout << "Slist :" << Slist << std::endl;

    Eigen::VectorXd thetaList(6);
    thetaList << qpos[0], qpos[1], qpos[2],
    qpos[3], qpos[4], qpos[5];
    Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

    // std::cout << "FKCals :" << std::endl;
    // std::cout << FKCal << std::endl;
    // std::cout  << std::endl; 
    
    // std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
    return FKCal;
}


Eigen::MatrixXd ur_frame::jacobian_space(const mjModel* m,mjData* d){
    //ur5
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist <<    0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -param.H1, -param.H1,  -param.H1,		-param.W1,	param.H2 - param.H1,
                0, 0,    0,	0,		param.L1 + param.L2,0,
                0, 0,    param.L1,	param.L1 + param.L2, 0,		 param.L1 + param.L2;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];
    Eigen::MatrixXd result = mr::JacobianSpace(Slist, thetaList);
    return result;
}


Eigen::MatrixXd ur_frame::jacobian_body(const mjModel* m,mjData* d){
 
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -param.H1, -param.H1,  -param.H1,		-param.W1,	param.H2 - param.H1,
                0, 0,    0,	0,		param.L1 + param.L2,0,
                0, 0,    param.L1,	param.L1 + param.L2, 0,		 param.L1 + param.L2;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];
    Eigen::MatrixXd result = mr::JacobianBody(Slist, thetaList);
    return result;
}

Eigen::VectorXd ur_frame::inverse_kinematics(const mjModel* m,mjData* d,Eigen::MatrixXd T){

    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, param.L1+ param.L2,
        0, 0, 1, param.W1+param.W2,
        0, 1, 0, param.H1-param.H2,
        0, 0, 0, 1;
    Eigen::MatrixXd SlistT(6, 6);
    //s矩阵
    //列排列，非行排列
    SlistT << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -param.H1, -param.H1,  -param.H1,		-param.W1,	param.H2 - param.H1,
                0, 0,    0,	0,		param.L1 + param.L2,0,
                0, 0,    param.L1,	param.L1 + param.L2, 0,		 param.L1 + param.L2;
    Eigen::MatrixXd Slist = SlistT;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];

    double eomg = 0.1;
    double ev = 0.004;
    bool b_result = true;
    // Eigen::VectorXd theta_result(3);
    // theta_result << 1.57073783, 2.99966384, 3.1415342;
    bool iRet = mr::IKinSpace(Slist, M, T, thetaList, eomg, ev);
    if (iRet)
    {
        // std::cout << "IKinSpace success" << std::endl;
        // std::cout << "thetaList :" << thetaList << std::endl;
        // for (size_t i = 0; i < m->nu; i++)
        // {
        //     std::cout << "  " << d->qpos[i];
        // }
        // std::cout <<std::endl;

        return thetaList;
        
    }
    else
    {
        std::cout << "IKinSpace failed" << std::endl;
    }
    
}


bool ur_frame::inverse_kinematics(const tf_t flange_pose, const vec3_t &sign_sln, const double &q6_des, std::vector<double> &sln)
{
	sln.resize(6);
	//
	double d6 = param.W2;
	double d5 = param.H2;
	double d4 = param.W1;
	double d1 = param.H1;
	double a3 = -param.L2;
	double a2 = -param.L1;

    double A = d6 * flange_pose(1, 2) - flange_pose(1, 3);
	double B = d6 * flange_pose(0, 2) - flange_pose(0, 3);
	double R = A * A + B * B;
	if (is_zero(A)) {
		double div;
		if (is_same(fabs(d4), fabs(B)))
			div = -SIGN(d4) * SIGN(B);
		else
			div = -d4 / B;
		double arcsin = asin(div);
		if (is_zero(arcsin))
			arcsin = 0.0;
		sln[0] = arcsin;
		if (sign_sln.x() < 0)
			sln[0] = M_PI - sln[0];
	}
	else if (is_zero(B)) {
		double div;
		if (is_same(fabs(d4), fabs(A)))
			div = SIGN(d4) * SIGN(A);
		else
			div = d4 / A;
		double arccos = acos(div);
		sln[0] = arccos;
		if (sign_sln.x() < 0)
			sln[0] = 2 * M_PI - arccos;
	}
	else if (d4 * d4 > R) {
		return false;
	}
	else {
		double arccos = acos(d4 / sqrt(R));
		double arctan = atan2(-B, A);
		double pn = sign_sln.x() * arccos + arctan;
		if (is_zero(pn))
			pn = 0.0;
		sln[0] = pn;
	}
	// 计算q5-腕2
	double numer = (flange_pose(0, 3) * sin(sln[0]) - flange_pose(1, 3) * cos(sln[0]) - d4);
	double div;
	if (is_same(fabs(numer), fabs(d6)))
		div = SIGN(numer) * SIGN(d6);
	else
		div = numer / d6;
	double arccos = acos(div);
	if (sign_sln.z() > 0)
		sln[4] = arccos;
	else
		sln[4] = -arccos;
	// 计算q6-腕3
	double c1 = cos(sln[0]), s1 = sin(sln[0]);
	double c5 = cos(sln[4]), s5 = sin(sln[4]);
	if (is_zero(s5)) {
		sln[5] = q6_des;
	}
	else {
		sln[5] = atan2(SIGN(s5) * -(flange_pose(0, 1) * s1 - flange_pose(1, 1) * c1), SIGN(s5) * (flange_pose(0, 0) * s1 - flange_pose(1, 0) * c1));
		if (fabs(sln[5]) < ZERO_THRESH)
			sln[5] = 0.0;
	}
	// 计算q2,q3,q4
	double c6 = cos(sln[5]), s6 = sin(sln[5]);
	double x04x = -s5 * (flange_pose(0, 2) * c1 + flange_pose(1, 2) * s1) -
				  c5 * (s6 * (flange_pose(0, 1) * c1 + flange_pose(1, 1) * s1) - c6 * (flange_pose(0, 0) * c1 + flange_pose(1, 0) * s1));
	double x04y = c5 * (flange_pose(2, 0) * c6 - flange_pose(2, 1) * s6) - flange_pose(2, 2) * s5;
	double p13x = d5 * (s6 * (flange_pose(0, 0) * c1 + flange_pose(1, 0) * s1) + c6 * (flange_pose(0, 1) * c1 + flange_pose(1, 1) * s1)) -
				  d6 * (flange_pose(0, 2) * c1 + flange_pose(1, 2) * s1) + flange_pose(0, 3) * c1 + flange_pose(1, 3) * s1;
	double p13y = flange_pose(2, 3) - d1 - d6 * flange_pose(2, 2) + d5 * (flange_pose(2, 1) * c6 + flange_pose(2, 0) * s6);

	double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
	if (is_same(fabs(c3), 1.0))
		c3 = SIGN(c3);
	else if (fabs(c3) > 1.0) {
		return false;
	}
	double arccos3 = acos(c3);
	sln[2] = sign_sln.y() > 0 ? arccos3 : -arccos3;
	double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
	double s3 = sin(arccos3);
	double AA = (a2 + a3 * c3), BB = a3 * s3;
	sln[1] = atan2((AA * p13y - sign_sln.y() * BB * p13x) / denom, (AA * p13x + sign_sln.y() * BB * p13y) / denom);
	double c23_0 = cos(sln[1] + sln[2]);
	double s23_0 = sin(sln[1] + sln[2]);
	sln[3] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);

	////////////////////////////////////////////////////////////////////////////////
	if (is_zero(sln[1]))
		sln[1] = 0.0;
	if (is_zero(sln[3]))
		sln[3] = 0.0;
	return true;
}

bool ur_frame::inverse_kinematics(const tf_t flange_pose, std::vector<std::vector<double>> &slns)
{
	std::vector<vec3_t> sign_slns = {{1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1}};
	slns.clear();
	std::vector<double> sln;
	for (int i = 0; i < sign_slns.size(); i++) {
		if (inverse_kinematics(flange_pose, sign_slns[i], 0, sln) && __is_within_limit(sln)) {
			slns.push_back(sln);
		}
	}
	return !slns.empty();
}


auto ur_frame::inverse_kinematics(const double *ee_pm, int which_root, double *input)->bool
{

    double a[6] = { 0, -param.L1, -param.L2, 0, 0, 0 };
    double d[6] = { param.H1, 0, 0, param.W1, param.H2, param.W2 };
    double nx = ee_pm[0], ny = ee_pm[4], nz = ee_pm[8];
    double ox = ee_pm[1], oy = ee_pm[5], oz = ee_pm[9];
    double ax = ee_pm[2], ay = ee_pm[6], az = ee_pm[10];
    double px = ee_pm[3], py = ee_pm[7], pz = ee_pm[11];
    double q[6]{ 0 };
    //求解关节角1//
    double m = d[5]*ay - py,  n = ax * d[5] - px;
    if ((m*m + n * n - d[3] * d[3]) < 0) return false;
    if (which_root & 0x04)
    {
        q[0] = atan2(m, n) - atan2(d[3], sqrt(m*m + n*n - d[3]*d[3]));
    }
    else
    {
        q[0] = atan2(m, n) - atan2(d[3], -sqrt(m*m + n * n - d[3] * d[3]));
    }
    
    //求解关节角5//
    if ((ax*sin(q[0]) - ay * cos(q[0])) > 1) return false;
    if (which_root & 0x02)
    {
        q[4] = acos(ax*sin(q[0]) - ay * cos(q[0]));
    }
    else
    {
        q[4] = -acos(ax*sin(q[0]) - ay * cos(q[0]));
    }
    
    //求解关节角6//
    double mm = nx * sin(q[0]) - ny * cos(q[0]);
    double nn = ox * sin(q[0]) - oy * cos(q[0]);
    q[5] = atan2(mm, nn) - atan2(sin(q[4]), 0); 
    //求解关节角3//
    double mmm = d[4]*(sin(q[5])*(nx*cos(q[0]) + ny * sin(q[0])) + cos(q[5])*(ox*cos(q[0]) + oy * sin(q[0])))
        - d[5]*(ax*cos(q[0]) + ay * sin(q[0])) + px * cos(q[0]) + py * sin(q[0]);
    double nnn = pz - d[0] - az * d[5] + d[4]*(oz*cos(q[5]) + nz * sin(q[5]));
    if ((mmm*mmm + nnn * nnn) > (a[1]*a[1]+a[2]*a[2]+2*a[1]*a[2])) return false;
    if (which_root & 0x01)
    {
    q[2] = acos((mmm*mmm + nnn*nnn - a[1]*a[1] - a[2]*a[2]) / (2 * a[1]*a[2]));
    }
    else
    {
        q[2] = -acos((mmm*mmm + nnn * nnn - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
    }
    
    //求解关节角2//

    double s2 = ((a[2]*cos(q[2]) + a[1])*nnn - a[2]*sin(q[2])*mmm)/
        (a[1]*a[1] + a[2]*a[2] + 2 * a[1]*a[2]*cos(q[2]));
    double c2 = (mmm + a[2]*sin(q[2])*s2)/ (a[2]*cos(q[2]) + a[1]);
    q[1] = atan2(s2, c2);

    //求解关节角4//
    q[3] = atan2(-sin(q[5])*(nx*cos(q[0]) + ny * sin(q[0])) - cos(q[5])*
        (ox*cos(q[0]) + oy * sin(q[0])), oz*cos(q[5]) + nz * sin(q[5])) - q[1] - q[2];

    // 添加所有的偏移 //
    for (int i = 0; i < 6; ++i)
    {
        while (q[i] > M_PI) q[i] -= 2 * M_PI;
        while (q[i] < -M_PI) q[i] += 2 * M_PI;
    }

    // 将q copy到input中
    s_vc(6, q, input);
    return true;
}


Eigen::VectorXd ur_frame::select_sln(const double *ee_pm,mjData* d)
{
    int solution_num = 0;
    double diff_q[8][6];
    double diff_norm[8];
    // double ans_pos[6];
    Eigen::VectorXd thetaList(6);
    std::cout<<"all_sln:"<<std::endl;
    for (int i = 0; i < 8; ++i)
    {
        if (inverse_kinematics(ee_pm, i, diff_q[solution_num]))
        {
            
            diff_norm[solution_num] = 0;
            for (int j = 0; j < 6; ++j)
            {
                std::cout<<diff_q[solution_num][j]<<"  ";

                diff_q[solution_num][j] -= d->qpos[j];
    
                while (diff_q[solution_num][j] > M_PI) diff_q[solution_num][j] -= 2 * M_PI;
                while (diff_q[solution_num][j] < -M_PI)diff_q[solution_num][j] += 2 * M_PI;
    
                diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
            }
            ++solution_num;
        }
        std::cout<<std::endl;
    }
    //if (solution_num == 0) return -1;
    auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;
    //选解策略（距离最近）
    //std::cout << "ik_pos :" << std::endl;
    for (uint j = 0; j < 6; ++j)
    {
        //std::cout <<" "<< d->qpos[j];
        thetaList[j] = d->qpos[j] + diff_q[real_solution][j];
        while (thetaList[j] > 2 * M_PI) thetaList[j] -= 2 * M_PI;
        while (thetaList[j] < -2 * M_PI)thetaList[j] += 2 * M_PI;
    }
    // std::cout<<std::endl;
    // std::cout << "ik_solution :" << std::endl;
    // // //std::cout<< ans_pos[0] <<"  "<< ans_pos[1] <<"  "<< ans_pos[2] <<"  "<< ans_pos[3] <<"  "<< ans_pos[4] <<"  "<< ans_pos[5] <<"  "<< std::endl;
    // std::cout<< thetaList[0] <<"  "<< thetaList[1] <<"  "<< thetaList[2] <<"  "<< thetaList[3] <<"  "<< thetaList[4] <<"  "<< thetaList[5] <<"  "<< std::endl;
    // std::cout<< d->qpos[0] <<"  "<< d->qpos[1] <<"  "<< d->qpos[2] <<"  "<< d->qpos[3] <<"  "<< d->qpos[4] <<"  "<< d->qpos[5] <<"  "<< std::endl;

    // s_vc(6, diff_q[real_solution], d->qpos);
    return thetaList;
}

bool ur_frame::select_sln(const tf_t ee_pm,mjData* d,std::vector<double> &sln){
    std::vector<std::vector<double>> slns;
    std::vector<double> cur_q = double2stdvec(d->qpos,6);

    if(inverse_kinematics(ee_pm,slns))
    {
        if(__sort_slns(cur_q,slns)){
            sln = slns[0];
            return true;
        }
    }
    return false;
}


bool ur_frame::__is_within_limit(const std::vector<double>& joint){
    if(joint.size()){
        for(int i=0; i<6;i++){
            if(joint[i]>2*M_PI||joint[i]<-2*M_PI)
                return false;
        }
        return true;
    }
}

bool ur_frame::__convert_sln(const std::vector<double> &joint,std::vector<double> &sln){
    if(!joint.size())
        return false;
    for(int i=0; i<sln.size();i++){
        double tmp = sln[i]<0?sln[i]+2*M_PI:sln[i]-2*M_PI;
        if(tmp<M_PI&&tmp>-M_PI&&std::abs(tmp - joint[i])<std::abs(sln[i]-joint[i])){
            sln[i]=tmp;
        }
    }
    return true;
}

bool ur_frame::__sort_slns(const std::vector<double> &joint,std::vector<std::vector<double>>  &slns){
    std::multimap<double, std::vector<double>> cost_slns;
    std::vector<std::vector<double>> tmp_slns = slns;
    std::cout<<"all_sln2"<<std::endl;
    for(auto &sln:slns){
        for(auto &i:sln)
            std::cout<<i<<"  ";
        std::cout<<std::endl;    
        if(!__convert_sln(joint,sln))
            return false;
        cost_slns.insert(std::make_pair(distance(joint,sln),sln));
    }
    slns.clear();
    for(const std::pair<double,std::vector<double>>& it:cost_slns)
        slns.push_back(it.second);
    return true;
}