
#include "FileYaml.h"


static void write(FileStorage& fs, const std::string&, const Camera& x) 
{
     x.write(fs);
}

static void read(const FileNode& node, Camera& x, const Camera& default_value = Camera()) 
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

void readConfig(const char *yamlPath,Camera_Other_Parameter &vecCamerasOtherParam)
{
    std::vector<Camera> vecCameras;
    cv::FileStorage fin(yamlPath, cv::FileStorage::READ);
    if(!fin.isOpened())
    {
	cerr << endl << "Failed to load readConfig yamlPath " << endl;
	return ;
    }
      
    cv::FileNode cameras_node = fin["cameras"];
/*    
    cv::FileNode Rl_node = fin["Rl"];
    cv::FileNode Rr_node = fin["Rr"];
    cv::FileNode Pl_node = fin["Pl"];
    cv::FileNode Pr_node = fin["Pr"];
    cv::FileNode Kl_node = fin["Kl"];
    cv::FileNode Kr_node = fin["Kr"];
    cv::FileNode Dl_node = fin["Dl"];
    cv::FileNode Dr_node = fin["Dr"];*/
    
    fin["Rl"] >> vecCamerasOtherParam.R_l;
    fin["Rr"] >> vecCamerasOtherParam.R_r;
    fin["Pl"] >> vecCamerasOtherParam.P_l;
    fin["Pr"] >> vecCamerasOtherParam.P_r;
    fin["Kl"] >> vecCamerasOtherParam.K_l;
    fin["Kr"] >> vecCamerasOtherParam.K_r;
    fin["Dl"] >> vecCamerasOtherParam.D_l;
    fin["Dr"] >> vecCamerasOtherParam.D_r;

 /*
    vecCamerasOtherParam.R_l = Rl_node.mat();
    vecCamerasOtherParam.R_r = Rr_node.mat();
    vecCamerasOtherParam.P_l = Pl_node.mat();
    vecCamerasOtherParam.P_r = Pr_node.mat();
    vecCamerasOtherParam.K_l = Kl_node.mat();
    vecCamerasOtherParam.K_r = Kr_node.mat();
    vecCamerasOtherParam.D_l = Dl_node.mat();
    vecCamerasOtherParam.D_r = Dr_node.mat();*/

    for (cv::FileNodeIterator it = cameras_node.begin(); it != cameras_node.end(); it++)
    {
        Camera camera;
        (*it) >> camera;
        vecCameras.push_back(camera);
    }

    //obtain col & row
    vecCamerasOtherParam.cols = vecCameras[0].imageDimension(0);
    vecCamerasOtherParam.rows = vecCameras[0].imageDimension(1);
    //obtain R & t
    Eigen::Matrix4d T_SC_left;
    Eigen::Matrix4d T_SC_right;
    T_SC_left  = vecCameras[0].T_SC;
    T_SC_right = vecCameras[1].T_SC;
    SE3 T_SC_l(T_SC_left.topLeftCorner(3,3),T_SC_left.topRightCorner(3,1));
    SE3 T_SC_r(T_SC_right.topLeftCorner(3,3),T_SC_right.topRightCorner(3,1));
    SE3 Tcl_cr = T_SC_l.inverse()*T_SC_r;
    SE3 Tcr_cl = T_SC_r.inverse()*T_SC_l;
    Matrix3d R = Tcr_cl.rotation_matrix();
    Vector3d t = Tcr_cl.translation();
    
    //Eigen tranfer to array
    double * R_ptr= new double[R.size()];
    double *t_ptr = new double[t.size()];
    Map<MatrixXd>(R_ptr, R.rows(), R.cols()) = R;
    Map<MatrixXd>(t_ptr, t.rows(), t.cols()) = t;
    cout<<"R_matrix"<<endl;
    double R_matrix[3][3];
    for(int i = 0;i < 3;i++)
      for(int j = 0;j<3;j++)
      {
	  //transpose
	  R_matrix[j][i] = R_ptr[i+j*3];
	  cout<<R_matrix[j][i]<<endl;
      }
       
    cout<<"t_matrix"<<endl;
    double t_matrix[3];
    for(int i = 0;i < 3;i++)
    {
	t_matrix[i] = t_ptr[i];
	cout<<t_matrix[i]<<endl;
    }
    vecCamerasOtherParam.R = cv::Mat(3,3,CV_64FC1,R_matrix);
    vecCamerasOtherParam.t = cv::Mat(3,1,CV_64FC1,t_matrix);
}


