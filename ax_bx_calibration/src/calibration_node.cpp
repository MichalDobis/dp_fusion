//
// Created by Miroslav Kohut on 7/20/17. Source edited and tested by Miroslav Kohut.
// Mathematic backround and integration to ceres was coded by Mojmir Majdis THANKS !!

//ROS
#include "ros/ros.h"

//STD
#include <iostream>
#include <fstream>   // std::cout
#include <fstream>

//CERES
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include "ceres_extensions.h"
//
#include <ceres/autodiff_cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/solver.h>

class CameraCalib{

public:

    CameraCalib(){
        std::string ns("/ax_bx_calibration/");
        if ((nh.getParam(ns+"input_file_path", inputpath)) && (nh.getParam(ns+"output_file_path", outputpath)))
        {
            nh.param<bool>(ns+"use_points", use_points, true);
            nh.param<bool>(ns+"use_photoneo_scaner", use_photoneo_scaner, true);
            kNumObservations = 0;
        }
        else{
            ROS_ERROR_STREAM("Please set input_file_path parameter and output_file_path parameter");
            ros::shutdown();
        }
    }
    ~CameraCalib(){

    }
    bool RobotSpaceCalibration();

private:
    //Set the number of frames (observations)
    ros::NodeHandle nh;
    int kNumObservations;
    bool use_points,use_photoneo_scaner;
    std::string inputpath;
    std::string outputpath;
    std::vector<Eigen::Quaterniond> ObservedBaseQuaternions,ObservedCameraQuaternions;
    std::vector<Eigen::Vector3d> ObservedBaseTranslations, ObservedCameraTranslations;

    struct RotationResidual {
        RotationResidual(const Eigen::Quaterniond CameraQuaternion, const Eigen::Quaterniond BaseQuaternion) :CamR(CameraQuaternion),
                                                                                                              BaseR(BaseQuaternion) {};
        template <typename T> bool operator() (
                const T* const TrackerRotationQuaternion,
                const T* const AbsoluteRotationQuaternion,
                T* residual) const {

            Eigen::Quaternion<T> TrackerRotation = Eigen::Map<const Eigen::Quaternion<T>>(TrackerRotationQuaternion);
            Eigen::Quaternion<T> Absol = Eigen::Map<const Eigen::Quaternion<T>>(AbsoluteRotationQuaternion);
            Eigen::Quaternion<T> MultipleTransformation;
            Eigen::Quaternion<T> Cam1(T(CamR.w()), T(CamR.x()), T(CamR.y()), T(CamR.z()));
            Eigen::Quaternion<T> Base1(T(BaseR.w()), T(BaseR.x()), T(BaseR.y()), T(BaseR.z()));

            MultipleTransformation = Base1 * TrackerRotation * Cam1;

            residual[0] = MultipleTransformation.x() - Absol.x();
            residual[1] = MultipleTransformation.y() - Absol.y();
            residual[2] = MultipleTransformation.z() - Absol.z();
            residual[3] = MultipleTransformation.w() - Absol.w();
            return true;
        }

    private:
        // Observations for a sample.
        Eigen::Quaterniond CamR;
        Eigen::Quaterniond BaseR;
    };

    struct TranslationResidual {
        TranslationResidual(const Eigen::Quaterniond BaseR, const Eigen::Vector3d BaseT, const Eigen::Vector3d CameraT) : BaseRotation(BaseR),
                                                                                                                          BaseTranslation(BaseT),
                                                                                                                          CameraTranslation(CameraT) {}
        template <typename T> bool operator() (
                const T* const TrackerRotationQuaternion,
                const T* const TrackerTranslationVector,
                const T* const AbsoluteTranslationVector,
                T* residual) const {

            Eigen::Quaternion<T> BaseRotation1(T(BaseRotation.w()), T(BaseRotation.x()), T(BaseRotation.y()), T(BaseRotation.z()));
            Eigen::Matrix<T, 3, 1> BaseTranslation1(T(BaseTranslation.x()), T(BaseTranslation.y()), T(BaseTranslation.z()));
            Eigen::Quaternion<T> TrackerRotation = Eigen::Map<const Eigen::Quaternion<T>>(TrackerRotationQuaternion);
            Eigen::Matrix<T, 3, 1> TrackerTranslation(TrackerTranslationVector[0], TrackerTranslationVector[1], TrackerTranslationVector[2]);
            Eigen::Matrix<T, 3, 1> AbsoluteTranslation(AbsoluteTranslationVector[0], AbsoluteTranslationVector[1], AbsoluteTranslationVector[2]);
            Eigen::Matrix<T, 3, 1> CameraTranslation1(T(CameraTranslation.x()), T(CameraTranslation.y()), T(CameraTranslation.z()));

            Eigen::Matrix< T, 3, 1 > FirstCor((BaseRotation1 * TrackerRotation)._transformVector(CameraTranslation1));
            Eigen::Matrix< T, 3, 1 > SecondCor(BaseRotation1._transformVector(TrackerTranslation));


            residual[0] = FirstCor[0] + SecondCor[0] + BaseTranslation1[0] - AbsoluteTranslation[0];
            residual[1] = FirstCor[1] + SecondCor[1] + BaseTranslation1[1] - AbsoluteTranslation[1];
            residual[2] = FirstCor[2] + SecondCor[2] + BaseTranslation1[2] - AbsoluteTranslation[2];
            return true;

        }

    private:
        // Observations for a sample.
        Eigen::Quaterniond BaseRotation;
        Eigen::Vector3d CameraTranslation;
        Eigen::Vector3d BaseTranslation;
    };
    bool readDataFromFile();
    void init_points();
    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d R);
    Eigen::Quaterniond rotateAndReturnQuad(const Eigen::Vector3d input_to_rotate,const Eigen::Vector3d input_which_rotates);


    };

void CameraCalib::init_points(){
    ObservedBaseQuaternions.resize(kNumObservations);
    ObservedCameraQuaternions.resize(kNumObservations);
    ObservedBaseTranslations.resize(kNumObservations);
    ObservedCameraTranslations.resize(kNumObservations);
}


Eigen::Vector3d CameraCalib::rotationMatrixToEulerAngles(Eigen::Matrix3d R) {


    float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular) {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    } else {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);

}

Eigen::Quaterniond CameraCalib::rotateAndReturnQuad(const Eigen::Vector3d input_to_rotate,const Eigen::Vector3d input_which_rotates){

    Eigen::Vector3d temp_vec;
    temp_vec.x()=input_to_rotate.x()+input_which_rotates.x();
    temp_vec.y()=input_to_rotate.y()+input_which_rotates.y();
    temp_vec.z()=input_to_rotate.z()+input_which_rotates.z();

    Eigen::AngleAxisd rollAngle((temp_vec.x()*M_PI) / 180, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle((temp_vec.y()*M_PI) / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle((temp_vec.z()*M_PI) / 180, Eigen::Vector3d::UnitX());

    return  rollAngle * yawAngle * pitchAngle;
}


bool CameraCalib::readDataFromFile(){


    std::ifstream input_file;
    input_file.open(inputpath);

    std::string unused;
    while (std::getline(input_file, unused) )
        ++kNumObservations;
    input_file.clear();
    input_file.seekg(0, std::ios::beg);

    if (use_points) {
        if(kNumObservations<2){
            ROS_ERROR_STREAM("PROBLEM WITH DATA");
            return false;
        }
        else{
            ROS_WARN_STREAM("NUMBER OF OBSERVATIONS " << kNumObservations);
            CameraCalib::init_points();
        }

        ROS_WARN_STREAM("USING POINTS FOR COMMUTING TRANSFORMATION");
        for (int i = 0; i < kNumObservations; ++i) {
            // reading points from file girst is tool0 to base second is position of observation frame in camera frame

            input_file >> ObservedBaseTranslations[i].x() >> ObservedBaseTranslations[i].y()
                    >> ObservedBaseTranslations[i].z();
            input_file >> ObservedBaseQuaternions[i].x() >> ObservedBaseQuaternions[i].y()
                    >> ObservedBaseQuaternions[i].z() >> ObservedBaseQuaternions[i].w();

            std::cout << i << ".TOOL TO BASE TRANSFORM:" << std::endl;
            std::cout << ObservedBaseTranslations[i].x() << " " << ObservedBaseTranslations[i].y() << " "
                      << ObservedBaseTranslations[i].z();
            std::cout << " " << ObservedBaseQuaternions[i].x() << " " << ObservedBaseQuaternions[i].y() << " "
                      << ObservedBaseQuaternions[i].z() << " " << ObservedBaseQuaternions[i].w() << std::endl<<std::endl;

            input_file >> ObservedCameraTranslations[i].x() >> ObservedCameraTranslations[i].y()
                    >> ObservedCameraTranslations[i].z();
            input_file >> ObservedCameraQuaternions[i].x() >> ObservedCameraQuaternions[i].y()
                    >> ObservedCameraQuaternions[i].z() >> ObservedCameraQuaternions[i].w();

            std::cout << i << ".DATA TO CAMERA SPACE TRANSFORM:" << std::endl;
            std::cout << ObservedCameraTranslations[i].x() << " " << ObservedCameraTranslations[i].y() << " "
                      << ObservedCameraTranslations[i].z();
            std::cout << " " << ObservedCameraQuaternions[i].x() << " " << ObservedCameraQuaternions[i].y() << " "
                      << ObservedCameraQuaternions[i].z() << " " << ObservedCameraQuaternions[i].w() << " " << std::endl
                      << std::endl;
            /*these three lines needs to be commented when you are using data from cameraspace .... when we are using data from photoneo scanner
            lines need to be uncomented*/

            if (use_photoneo_scaner){

                ObservedCameraTranslations[i] = ObservedCameraTranslations[i] ;/// 1000;
                ObservedCameraQuaternions[i] = ObservedCameraQuaternions[i].conjugate();
                ObservedCameraTranslations[i] = -ObservedCameraQuaternions[i]._transformVector(ObservedCameraTranslations[i]);

            }


        }
    }
    else{

        //first base point init
        ObservedBaseTranslations[0].x() = 0;
        ObservedBaseTranslations[0].y() = 0;
        ObservedBaseTranslations[0].z() = 0;
        ObservedBaseQuaternions[0].x() = 0;
        ObservedBaseQuaternions[0].y() = 0;
        ObservedBaseQuaternions[0].z() = 0;
        ObservedBaseQuaternions[0].w() = 1;
        //first camera point init
        ObservedCameraTranslations[0].x()=1;
        ObservedCameraTranslations[0].y()=1;
        ObservedCameraTranslations[0].z()=1;
        ObservedCameraQuaternions[0].x()=0;
        ObservedCameraQuaternions[0].y()=0;
        ObservedCameraQuaternions[0].z()=0;
        ObservedCameraQuaternions[0].w()=1;

        kNumObservations = kNumObservations / 8;
        if(kNumObservations<2){
            ROS_ERROR_STREAM("NOT ENOUGH DATA");
            return false;
        }
        else{
            ROS_WARN_STREAM("NUMBER OF OBSERVATIONS " << kNumObservations);
            CameraCalib::init_points();
        }

        ROS_WARN_STREAM("USING MATRIXES A AND B FOR COMMUTING TRANSFORMATION");
            for(int i=1;i<kNumObservations;i++){
                // base transformations
                Eigen::Matrix4f base_world_transform_A;
                Eigen::Matrix3d rot_matrix_A;
                Eigen::Vector3d TempRotVec_A;
                Eigen::Vector3d TempRotVec_A2;
                //reading matrix A from file
                for(int x =0;x<4;x++){
                    for(int y =0;y<4;y++){
                        input_file >> base_world_transform_A(x,y);
                    }
                }
                ObservedBaseTranslations[i].x()=ObservedBaseTranslations[i-1].x()+base_world_transform_A(0,3);
                ObservedBaseTranslations[i].y()=ObservedBaseTranslations[i-1].y()+base_world_transform_A(1,3);
                ObservedBaseTranslations[i].z()=ObservedBaseTranslations[i-1].z()+base_world_transform_A(2,3);

                for(int a =0;a<3;a++){
                    for(int b =0;b<3;b++){
                        rot_matrix_A(a,b) = base_world_transform_A(a,b);
                    }
                }
                TempRotVec_A = CameraCalib::rotationMatrixToEulerAngles(rot_matrix_A);
                TempRotVec_A2 = CameraCalib::rotationMatrixToEulerAngles(ObservedBaseQuaternions[i-1].toRotationMatrix());
                ObservedBaseQuaternions[i]=CameraCalib::rotateAndReturnQuad(TempRotVec_A2,TempRotVec_A);

                // CAMERA TRANSFORMATIONS
                Eigen::Matrix3d rot_matrix_B;
                Eigen::Vector3d TempRotVec_B;
                Eigen::Vector3d TempRotVec_B2;
                Eigen::Matrix4f camera_space_point_transform_B;

                for(int x =0;x<4;x++) {
                    for (int y = 0; y < 4; y++) {
                        input_file >> camera_space_point_transform_B(x, y);
                    }
                }
                ObservedCameraTranslations[i].x()=ObservedCameraTranslations[i-1].x()+camera_space_point_transform_B(0,3);
                ObservedCameraTranslations[i].y()=ObservedCameraTranslations[i-1].y()+camera_space_point_transform_B(1,3);
                ObservedCameraTranslations[i].z()=ObservedCameraTranslations[i-1].z()+camera_space_point_transform_B(2,3);

                for(int a =0;a<3;a++){
                    for(int b =0;b<3;b++){
                        rot_matrix_B(a,b) = camera_space_point_transform_B(a,b);
                    }
                }

                TempRotVec_B = CameraCalib::rotationMatrixToEulerAngles(rot_matrix_B);
                TempRotVec_B2 = CameraCalib::rotationMatrixToEulerAngles(ObservedCameraQuaternions[i-1].toRotationMatrix());
                ObservedCameraQuaternions[i]=CameraCalib::rotateAndReturnQuad(TempRotVec_B2,TempRotVec_B);
                //reading matrix B from file

                ROS_INFO_STREAM(base_world_transform_A);
                ROS_INFO_STREAM(camera_space_point_transform_B);
            }
        return false;
    }
    input_file.close();
    return true;

}

bool CameraCalib::RobotSpaceCalibration(){

    if(!CameraCalib::readDataFromFile())
        return false;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 1000;
    options.function_tolerance = 1e-10;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    ceres::Solver::Summary summary;

    ceres::Problem problem;

    ceres::LocalParameterization *quaternion_parameterization = new ceres_ext::EigenQuaternionParameterization;

    double TrackerRotationQuaternion[4] = { 0, 0, 0, 1 };
    double AbsoluteRotationQuaternion[4] = { 0, 0, 0, 1 };
    double TrackerTranslationVector[3] = { 0, 0, 0 };
    double AbsoluteTranslationVector[3] = { 0, 0, 0 };

    for (int i = 0; i < kNumObservations; ++i) {
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<RotationResidual, 4, 4, 4>(
                        new RotationResidual(ObservedCameraQuaternions[i], ObservedBaseQuaternions[i]));
        problem.AddResidualBlock(cost_function, NULL, TrackerRotationQuaternion, AbsoluteRotationQuaternion);
    }


    for (int i = 0; i < kNumObservations; ++i){
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<TranslationResidual, 3, 4, 3, 3>(
                        new TranslationResidual(ObservedBaseQuaternions[i], ObservedBaseTranslations[i], ObservedCameraTranslations[i]));
        problem.AddResidualBlock(cost_function, NULL, TrackerRotationQuaternion, TrackerTranslationVector, AbsoluteTranslationVector);
    }

    problem.SetParameterization(TrackerRotationQuaternion, quaternion_parameterization);
    problem.SetParameterization(AbsoluteRotationQuaternion, quaternion_parameterization);

    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";

    Eigen::Quaterniond RobotRQuaternion(TrackerRotationQuaternion[3], TrackerRotationQuaternion[0], TrackerRotationQuaternion[1], TrackerRotationQuaternion[2]);
    Eigen::Matrix3d RobotRMat(RobotRQuaternion.toRotationMatrix());

    std::cout <<RobotRMat <<std::endl;



    Eigen::Vector3d RobotTranslationVec;
    Eigen::Vector3d RobotRotationVec;
    RobotRotationVec = CameraCalib::rotationMatrixToEulerAngles(RobotRMat);
    RobotTranslationVec.x() = TrackerTranslationVector[0];
    RobotTranslationVec.y() = TrackerTranslationVector[1];
    RobotTranslationVec.z() = TrackerTranslationVector[2];


    std::cout << "Translation" << "\n";
    std::cout << RobotTranslationVec << "\n";
    std::cout << "Rotation" << "\n";
    std::cout << RobotRotationVec << "\n";


    std::ofstream output_file(outputpath,std::ofstream::out);

    output_file << "Translation(m)" << "\n";
    output_file << RobotTranslationVec << "\n";
    output_file<< "Rotation(rad)" << "\n";
    output_file << RobotRotationVec << "\n";

    //std::cout << RobotRQuaternion.coeffs() << "\n";
    //result rotation = RobotRMat (as Eigen::Matrix3D), rotation = RobotRQuaternion (as Eigen::Quaterniond) , translation = RobotTranslationVec (as Eigen::Vector3D), translation = TrackerTranslationVector (as double[3])

    output_file.close();
    return true;
}

int main(int argc,char** argv){

    ros::init(argc, argv, "calibration_node");

    CameraCalib cameraCalib;
    cameraCalib.RobotSpaceCalibration();
    ROS_INFO_STREAM("PRESS ENTER TO EXIT PROGRAM");
    getchar();

}