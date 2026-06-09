#include "utils.hpp"


void readConfig(const std::string& cfg_filepath, Config& out_cfg)
{
    const YAML::Node config = YAML::LoadFile(cfg_filepath);

    // Filter parameters
    out_cfg.dataset = config["dataset"].as<std::string>();
    out_cfg.output = config["output"].as<std::string>();
    out_cfg.canonic_inliers = config["canonic_inliers"].as<int>();
    out_cfg.maxiters = config["max_iters"].as<int>();
    out_cfg.inlier_th = config["inlier_th"].as<double>();
    out_cfg.alpha = config["alpha"].as<double>();
    out_cfg.init_loop = config["init_loop"].as<bool>();

    return;
}

void store3D(const std::string& output_filepath, const gtsam::Values& result)
{
    std::ofstream outfile;
    outfile.open(output_filepath.c_str());
    for (int counter = 0; counter < result.size(); ++counter )
    {
        gtsam::Pose3 opt_pose = result.at<gtsam::Pose3>(gtsam::Key(counter));
        gtsam::Rot3 rot = opt_pose.rotation();
        gtsam::Vector3 angles = rot.rpy();
        outfile << opt_pose.x() << " " << opt_pose.y() << " " << opt_pose.z() << " "
                << angles[0]    << " " << angles[1]    << " " << angles[2] << std::endl;
    }
    outfile.close();
}

void store2D(const std::string& output_filepath, const gtsam::Values& result)
{
    std::ofstream outfile;
    outfile.open(output_filepath.c_str());
    for (int counter = 0; counter < result.size(); ++counter )
    {
        gtsam::Pose2 opt_pose = result.at<gtsam::Pose2>(gtsam::Key(counter));
        outfile << opt_pose.x() << " " << opt_pose.y() << " " << opt_pose.theta() << std::endl;
    }
    outfile.close();
}

void addPrior3D(gtsam::NonlinearFactorGraph& nfg)
{
    typedef Eigen::Matrix<double, 1, 6> Vector6d;
    Vector6d prior_noise = Vector6d::Ones();
    prior_noise.block<1,3>(0, 0) *= 0.001;
    prior_noise.block<1,3>(0, 3) *= 0.0003; 
    auto priorModel = gtsam::noiseModel::Diagonal::Sigmas(prior_noise);
    nfg.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), priorModel));

    return;
}

void addPrior2D(gtsam::NonlinearFactorGraph& nfg)
{
    const Eigen::Vector3d prior_noise(1e-6, 1e-6, 1e-8);
    auto priorModel = gtsam::noiseModel::Diagonal::Variances(prior_noise);
    nfg.add(gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(), priorModel));
}
