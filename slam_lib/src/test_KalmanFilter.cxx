#include "KalmanFilter.h"
#include <random>
#include <fstream>
#include <string>
#include <iostream>

void save(std::vector<Eigen::Vector3d>& pos, std::string file_name)
{
    // Save XYZ positions in file_name
    std::ofstream fout(file_name);
    if(fout.is_open())
    {
        for(int i = 0; i<pos.size(); ++i)
        {
            for(int j = 0; j<pos[i].rows()-1; ++j)
                fout<<pos[i](j)<<" ";
            fout<<pos[i][pos[i].rows()-1]<<"\n";
        }
        fout.close();
    }
}

struct normal_random_variable
{
    //Structure used to create a sample from a Gaussian distribution
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
    }
};

void Measure(inputMeasure& s, Eigen::MatrixXd x)
{
    // Simulate the output of sensor s in current state x
    normal_random_variable sample(s.MeasureCovariance);
    s.Measure = s.MeasureModel * x + sample();
}

int main()
{
    // One object is supposed to be moving around a circle on XY plane and Z = 1
    // We simulate two sensors measuring some elements of the object pose
    // We try to recover the real poses with a Kalman filter

    int N = 300; // number of points taken
    float delta_theta = 2*M_PI/N; // angular displacement at each step

    // Create real data
    std::vector<Eigen::Matrix<double, 12, 1>> real(N);
    for(int i = 0; i<N; ++i)
    real[i] << 0, 0, M_PI/2 + i*delta_theta, cos(i*delta_theta) , sin(i*delta_theta), 1, 0, 0, M_PI/2 + i*delta_theta, cos(i*delta_theta) , sin(i*delta_theta), 0;

    // Init first sensor (registration like?)
    inputMeasure s1;
    s1.MeasureModel = Eigen::MatrixXd::Identity(6, 12);
    Eigen::Matrix<double, 6, 1> diag_C1;
    diag_C1<<0.0, 0.0, pow(5*M_PI/180,2), pow(0.05,2), pow(0.02,2), pow(0.01,2);
    s1.MeasureCovariance  = Eigen::Matrix<double, 6, 6>(diag_C1.asDiagonal());

    // Init second sensor (GPS like?)
    inputMeasure s2;
    s2.MeasureModel = Eigen::MatrixXd::Zero(3, 12);
    s2.MeasureModel(0, 3) = 1;
    s2.MeasureModel(1, 4) = 1;
    s2.MeasureModel(2, 5) = 1;
    Eigen::Matrix<double, 3, 1> diag_C2;
    diag_C2<<pow(0.05,2), pow(0.05,2), pow(0.05,2);
    s2.MeasureCovariance  = Eigen::Matrix<double, 3, 3>(diag_C2.asDiagonal());

    // Use Kalman filter
    KalmanFilter kf;
    kf.ResetKalmanFilter();
    kf.InitState(real[0], Eigen::Matrix<double, 12, 12>::Zero());
    std::vector<inputMeasure*> sensors ={&s1, &s2};

    //------------------display purpose-------------------
    std::vector<Eigen::Vector3d> measures_1(N-1);
    std::vector<Eigen::Vector3d> measures_2(N-1);
    std::vector<Eigen::Vector3d> estimated(N-1);
    std::vector<Eigen::Vector3d> X(N-1);
    std::vector<Eigen::Matrix<double, 12, 1>> error(N-1);
    //----------------------------------------------------
    for(int i = 1; i<N; ++i)
    {
        Measure(s1, real[i]);
        Measure(s2, real[i]);
        kf.KalmanIteration(sensors, i); // i is supposed to be the time

        //------------------display purpose-------------------
        measures_1[i-1] = s1.Measure.block(3, 0, 3, 1);
        measures_2[i-1] = s2.Measure;
        estimated[i-1] = kf.GetState().block(3, 0, 3, 1);
        X[i-1] = real[i].block(3, 0, 3, 1);
        error[i-1] = kf.GetState()  - real[i];
        //----------------------------------------------------
    }

    // Compute some result to evaluate efficiency of Kalman

    Eigen::Matrix<double, 12, 1> std_dev;
    std_dev.setZero();
    Eigen::Matrix<double, 12, 1> mean;
    mean.setZero();
    for (int i=0; i<error.size(); ++i)
      mean += error[i];
    mean /= error.size();
    for (int i=0; i<error.size(); ++i)
      for(int j = 0; j<error[i].rows(); ++j)
        std_dev(j,0) += pow((error[i](j)-mean(j)),2);
    std_dev /= error.size();
    for(int j = 0; j<std_dev.rows(); ++j)
    std_dev(j,0) = sqrt(std_dev(j,0));

    std::cout<<"\nFirst sensor features :"<<std::endl;
    std::cout<<"\t std_deviation X = "<< sqrt(s1.MeasureCovariance(3, 3))<<"\t std_deviation Y = "<<sqrt(s1.MeasureCovariance(4, 4))<<"\t std_deviation Z = "<<sqrt(s1.MeasureCovariance(5, 5))<< "\t std_deviation alpha = "<<sqrt(s1.MeasureCovariance(2, 2)*180/M_PI)<<"°\n"<<std::endl;
    std::cout<<""<<std::endl;
    std::cout<<"Second sensor features :"<<std::endl;
    std::cout<<"\t std_deviation X = "<< sqrt(s2.MeasureCovariance(0, 0))<<"\t std_deviation Y = "<<sqrt(s2.MeasureCovariance(1, 1))<<"\t std_deviation Z = "<<sqrt(s2.MeasureCovariance(2, 2))<<"\n"<<std::endl;
    std::cout<<""<<std::endl;
    std::cout<<"Results : "<<std::endl;
    std::cout<<"\t mean_error X = "<< mean(3)<<"\t mean_error Y = "<<mean(4)<<"\t mean_error Z = "<<mean(5)<<"\t mean_error alpha = "<<mean(2)<<"°\n"<<std::endl;
    std::cout<<"\t std_deviation X = "<<std_dev(3)<<" \t std_deviation Y = "<<std_dev(4)<<" \t std_deviation Z = "<<std_dev(5)<<" \t std_deviation alpha = "<<std_dev(2)<<"°"<<std::endl;

    // Save XYZ position results

    save(X, "real.csv");
    save(estimated, "estimated.csv");
    save(measures_1, "sensor1.csv");
    save(measures_2, "sensor2.csv");

    return 0;
}
