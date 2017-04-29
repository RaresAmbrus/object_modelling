#ifndef __CERES_VIEW_REGISTRATION__
#define __CERES_VIEW_REGISTRATION__

#include <metaroom_xml_parser/load_utilities.h>
#include <ceres/ceres.h>
#include <siftgpu/SiftGPU.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/frustum_culling.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include "rgbd_view_registration_server/sift_wrapper.h"
#include "rgbd_view_registration_server/rgbd_view_registration_residual.h"
#include <pcl/visualization/pcl_visualizer.h>

class RGBDViewRegistrationOptimizer{

public:
    RGBDViewRegistrationOptimizer(bool verbose=false);
    ~RGBDViewRegistrationOptimizer();

    template <class PointType>
    bool registerViews(const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& all_views, const std::vector<tf::StampedTransform>& all_initial_poses,
                       std::vector<int>& number_of_constraints, std::vector<tf::Transform>& registered_poses){
        using namespace std;
        using namespace cv;
        using namespace ceres;

        vector<Mat> vRGBImages;
        vector<Mat> vDepthImages;

        // set up initial data
        for (auto view : all_views){
            auto image_pair = createRGBandDepthFromCloud(view);
            vRGBImages.push_back(image_pair.first);
            vDepthImages.push_back(image_pair.second);
        }

        // extract SIFT keypoints and descriptors
        struct SIFTData{
            int image_number;
            int desc_number;
            vector<float> descriptors;
            vector<SiftGPU::SiftKeypoint> keypoints;
        };

        SIFTWrapper sift_wrapper;
        vector<SIFTData> vSIFTData;

        for (size_t i=0; i<vRGBImages.size();i++){
            auto image = vRGBImages[i];
            SIFTData sift;
            sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
            vSIFTData.push_back(sift);
            if (m_bVerbose){
                ROS_INFO_STREAM("Extracted "<<vSIFTData[vSIFTData.size()-1].keypoints.size()<<" SIFT keypoints for image "<<i);
            }
        }

        // set up constraint problem
        struct ConstraintStructure{
            int image1;
            int image2;
            double depth_threshold;
            vector<pair<PointType, PointType>> correspondences;
            vector<pair<cv::Point2d, cv::Point2d>> correspondences_2d;
        };
        vector<ConstraintStructure> constraints_and_correspondences;

        for (size_t i=0; i<all_views.size()-1; i++){
            for (size_t j=i+1; j<all_views.size(); j++){
                ConstraintStructure constr;
                constr.image1 = i;
                constr.image2 = j;
                if (j!=i+1){
                    constr.depth_threshold = 1.5; // 5.0
                } else {
                    constr.depth_threshold = 1.5; // 5.0; //3.0; // 1.5
                }
                constraints_and_correspondences.push_back(constr);
            }
        }

        // get and validate sift matches
        for (ConstraintStructure& constr: constraints_and_correspondences){
            SIFTData image1_sift = vSIFTData[constr.image1];
            SIFTData image2_sift = vSIFTData[constr.image2];
            vector<pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>> matches;

            sift_wrapper.matchSIFT(image1_sift.desc_number, image2_sift.desc_number,
                                   image1_sift.descriptors, image2_sift.descriptors,
                                   image1_sift.keypoints, image2_sift.keypoints,
                                   matches);
            if (m_bVerbose){
                ROS_INFO_STREAM("Resulting number of matches for images "<<constr.image1<<" "<<constr.image2<<" is "<<matches.size());
            }
            constr.correspondences = validateMatches(matches,
                                                     vRGBImages[constr.image1], vRGBImages[constr.image2],
                                                     all_views[constr.image1], all_views[constr.image2],
                                                     constr.depth_threshold,
                                                     constr.correspondences_2d);
            if (m_bVerbose){
                ROS_INFO_STREAM("After validating "<<constr.correspondences.size()<<"  "<<constr.correspondences_2d.size()<<" are left ");
            }
        }

        // set up ceres problem
        struct Camera{
            double quaternion[4] = {1.0,0.0,0.0,0.0};
            double translation[3] = {0.0,0.0,0.0};
            Camera(){};
        };

        // ceres setup
        Problem problem;
        Solver::Options options;
        options.function_tolerance = 1e-30;
        options.parameter_tolerance = 1e-20;
        options.max_num_iterations = 100000;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
//        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 8;
        options.num_linear_solver_threads = 8;
        double min_correspondences = 12;


        vector<Camera*>  all_cameras;
        // initial solution
        for (size_t i=0; i<all_views.size(); i++){
            Camera* cam = new Camera();

            if (all_initial_poses.size() == all_views.size()){
//             if (all_initial_poses.size() > i){
                cam->translation[0] = all_initial_poses[i].getOrigin().x();
                cam->translation[1] = all_initial_poses[i].getOrigin().y();
                cam->translation[2] = all_initial_poses[i].getOrigin().z();
                cam->quaternion[0]  = all_initial_poses[i].getRotation().w();
                cam->quaternion[1]  = all_initial_poses[i].getRotation().x();
                cam->quaternion[2]  = all_initial_poses[i].getRotation().y();
                cam->quaternion[3]  = all_initial_poses[i].getRotation().z();
            }

            all_cameras.push_back(cam);
            problem.AddParameterBlock(cam->quaternion,4);
            problem.AddParameterBlock(cam->translation,3);

            number_of_constraints.push_back(0);

//            if (all_initial_poses.size() > i){
//                problem.SetParameterBlockConstant(cam->quaternion);
//                problem.SetParameterBlockConstant(cam->translation);
//            }
        }

        // cost functions
        vector<CostFunction*> rosrallCFs;
        vector<LossFunction*> allLFs;
        int total_constraints = 0;

        for (auto constr : constraints_and_correspondences){
            Camera* cam1 = all_cameras[constr.image1];
            Camera* cam2 = all_cameras[constr.image2];

            if (constr.correspondences.size() < min_correspondences){
                continue;
            }

            for (size_t k=0; k<constr.correspondences.size(); k++){
                auto corresp = constr.correspondences[k];
                auto corresp_2d = constr.correspondences_2d[k];
                double point_original1[3] = {corresp.first.x, corresp.first.y, corresp.first.z};
                double point_original2[3] = {corresp.second.x, corresp.second.y, corresp.second.z};

                double point_original1_2d[2] = {corresp_2d.first.x, corresp_2d.first.y};
                double point_original2_2d[2] = {corresp_2d.second.x, corresp_2d.second.y};
                double depth1 = corresp.first.z;
                double depth2 = corresp.second.z;

                double weight = (corresp.first.getVector3fMap().squaredNorm() + corresp.second.getVector3fMap().squaredNorm())/2;

                ceres::CostFunction *cost_function_proj = ProjectionResidual::Create(point_original1, point_original2, weight);
                ceres::LossFunction *loss_function_proj = new ceres::HuberLoss(weight * 0.005);
                problem.AddResidualBlock(cost_function_proj,
                                         loss_function_proj,
                                         cam1->quaternion,
                                         cam1->translation,
                                         cam2->quaternion,
                                         cam2->translation);
                total_constraints++;
            }
            number_of_constraints[constr.image1] += constr.correspondences.size();
            number_of_constraints[constr.image2] += constr.correspondences.size();
        }

        if (!all_cameras.size()){
            ROS_WARN_STREAM("RGBDViewRegistrationOptimizer ---- WARNING - no cameras defined for this object");
        }

        problem.SetParameterBlockConstant(all_cameras[0]->quaternion);
        problem.SetParameterBlockConstant(all_cameras[0]->translation);

        Solver::Summary summary;
        Solve(options, &problem, &summary);
        if (m_bVerbose){
            ROS_INFO_STREAM(summary.FullReport() << "\n");
        }

        ROS_INFO_STREAM("RGBD view regitration constraints "<<total_constraints);

        for (size_t i=0; i<all_cameras.size();i++){
            tf::Quaternion tf_q(all_cameras[i]->quaternion[1],all_cameras[i]->quaternion[2],all_cameras[i]->quaternion[3], all_cameras[i]->quaternion[0]);
            tf::Vector3 tf_v(all_cameras[i]->translation[0], all_cameras[i]->translation[1], all_cameras[i]->translation[2]);
            tf::Transform ceres_transform(tf_q, tf_v);
            registered_poses.push_back(ceres_transform);
        }

        // free memory
        // TODO check if this is necessary
        //        for (size_t i=0; i<allCFs.size();i++){
        //            delete allCFs[i];
        //        }
        //        for (size_t i=0; i<allLFs.size();i++){
        //            delete allLFs[i];
        //        }

        return true;
    }


    template <class PointType>
        bool registerViewsWithReference(const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& all_views,
                                        const std::vector<tf::StampedTransform>& all_initial_poses,
                                        const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& rgbd_views,
                                        const std::vector<tf::StampedTransform>& rgbd_initial_poses,
                           std::vector<int>& number_of_constraints, std::vector<tf::Transform>& registered_poses){
            using namespace std;
            using namespace cv;
            using namespace ceres;

            // first check that transforms have been provided for the reference views
            if (all_initial_poses.size() != all_views.size()){
                ROS_ERROR_STREAM("Number of poses for reference frames is != number of reference frames. ABORTING.");
                return true;
            }

            // create temp array with reference views transformed to map frame
            std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_views_map_frame;
            for (size_t i=0; i<all_views.size();i++){
                boost::shared_ptr<pcl::PointCloud<PointType>> temp_cloud (new pcl::PointCloud<PointType>);
                pcl_ros::transformPointCloud(*all_views[i], *temp_cloud,all_initial_poses[i]);
                all_views_map_frame.push_back(temp_cloud);
            }

            vector<Mat> vRGBImages;
            vector<Mat> vDepthImages;

            vector<Mat> regRGBImages;
            vector<Mat> regDepthImages;

            // set up initial data
            for (auto view : all_views){
                auto image_pair = createRGBandDepthFromCloud(view);
                vRGBImages.push_back(image_pair.first);
                vDepthImages.push_back(image_pair.second);
            }

            for (auto view : rgbd_views){
                auto image_pair = createRGBandDepthFromCloud(view);
                regRGBImages.push_back(image_pair.first);
                regDepthImages.push_back(image_pair.second);
            }

            // extract SIFT keypoints and descriptors
            struct SIFTData{
                int image_number;
                int desc_number;
                vector<float> descriptors;
                vector<SiftGPU::SiftKeypoint> keypoints;
            };

            SIFTWrapper sift_wrapper;
            vector<SIFTData> vSIFTData;

            for (size_t i=0; i<vRGBImages.size();i++){
                auto image = vRGBImages[i];
                SIFTData sift;
                sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
                vSIFTData.push_back(sift);
                if (m_bVerbose){
                    ROS_INFO_STREAM("Extracted "<<vSIFTData[vSIFTData.size()-1].keypoints.size()<<" SIFT keypoints for image "<<i);
                }
            }

            vector<SIFTData> regSIFTData;
            for (size_t i=0; i<regRGBImages.size();i++){
                auto image = regRGBImages[i];
                SIFTData sift;
                sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
                regSIFTData.push_back(sift);
                if (m_bVerbose){
                    ROS_INFO_STREAM("Extracted "<<regSIFTData[regSIFTData.size()-1].keypoints.size()<<" SIFT keypoints for image "<<i);
                }
            }

            // set up constraint problem
            struct ConstraintStructure{
                int image1;
                int image2;
                double depth_threshold;
                vector<pair<PointType, PointType>> correspondences;
                vector<pair<cv::Point2d, cv::Point2d>> correspondences_2d;
            };
            vector<ConstraintStructure> constraints_and_correspondences;
            vector<ConstraintStructure> rgbd_view_constraints_and_correspondences;

            for (size_t i=0; i<rgbd_views.size(); i++){
                for (size_t j=0; j<all_views.size(); j++){
                    ConstraintStructure constr;
                    constr.image1 = i;
                    constr.image2 = j;
                    if (j!=i+1){
                        constr.depth_threshold = 1.5; // 5.0
                    } else {
                        constr.depth_threshold = 1.5; // 5.0; //3.0; // 1.5
                    }
                    constraints_and_correspondences.push_back(constr);
                }
            }

            for (size_t i=0; i<rgbd_views.size()-1; i++){
                for (size_t j=i+1; j<rgbd_views.size(); j++){
                    ConstraintStructure constr;
                    constr.image1 = i;
                    constr.image2 = j;
                    if (j!=i+1){
                        constr.depth_threshold = 1.5; // 5.0
                    } else {
                        constr.depth_threshold = 1.5; // 5.0; //3.0; // 1.5
                    }
                    rgbd_view_constraints_and_correspondences.push_back(constr);
                }
            }

            // get and validate sift matches from rgbd views to reference frames
            for (ConstraintStructure& constr: constraints_and_correspondences){
                SIFTData image1_sift = regSIFTData[constr.image1];
                SIFTData image2_sift = vSIFTData[constr.image2];
                vector<pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>> matches;

                sift_wrapper.matchSIFT(image1_sift.desc_number, image2_sift.desc_number,
                                       image1_sift.descriptors, image2_sift.descriptors,
                                       image1_sift.keypoints, image2_sift.keypoints,
                                       matches);
                if (m_bVerbose){
                    ROS_INFO_STREAM("Resulting number of matches for images "<<constr.image1<<" "<<constr.image2<<" is "<<matches.size());
                }
                constr.correspondences = validateMatches(matches, regRGBImages[constr.image1], vRGBImages[constr.image2],
                                                         rgbd_views[constr.image1], all_views_map_frame[constr.image2],
                                                         constr.depth_threshold,
                                                         constr.correspondences_2d);
                if (m_bVerbose){
                    ROS_INFO_STREAM("After validating "<<constr.correspondences.size()<<"  "<<constr.correspondences_2d.size()<<" are left ");
                }
            }

            // get and validate sift matches from rgbd views to rgbd views
            for (ConstraintStructure& constr: rgbd_view_constraints_and_correspondences){
                SIFTData image1_sift = regSIFTData[constr.image1];
                SIFTData image2_sift = regSIFTData[constr.image2];
                vector<pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>> matches;

                sift_wrapper.matchSIFT(image1_sift.desc_number, image2_sift.desc_number,
                                       image1_sift.descriptors, image2_sift.descriptors,
                                       image1_sift.keypoints, image2_sift.keypoints,
                                       matches);
                if (m_bVerbose){
                    ROS_INFO_STREAM("Resulting number of matches for images "<<constr.image1<<" "<<constr.image2<<" is "<<matches.size());
                }
                constr.correspondences = validateMatches(matches, regRGBImages[constr.image1], regRGBImages[constr.image2],
                                                         rgbd_views[constr.image1], rgbd_views[constr.image2],
                                                         constr.depth_threshold,
                                                         constr.correspondences_2d);
                if (m_bVerbose){
                    ROS_INFO_STREAM("After validating "<<constr.correspondences.size()<<"  "<<constr.correspondences_2d.size()<<" are left ");
                }
            }

            // set up ceres problem
            struct Camera{
                double quaternion[4] = {1.0,0.0,0.0,0.0};
                double translation[3] = {0.0,0.0,0.0};
                Camera(){};
            };

            // ceres setup
            Problem problem;
            Solver::Options options;
            options.function_tolerance = 1e-30;
            options.parameter_tolerance = 1e-20;
            options.max_num_iterations = 100000;
            options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    //        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            options.num_threads = 8;
            options.num_linear_solver_threads = 8;
            double min_correspondences = 12;


            vector<Camera*>  reg_cameras;
            // initial solution
            for (size_t i=0; i<rgbd_views.size(); i++){
                Camera* cam = new Camera();

                if (rgbd_initial_poses.size() == rgbd_views.size()){
                    cam->translation[0] = rgbd_initial_poses[i].getOrigin().x();
                    cam->translation[1] = rgbd_initial_poses[i].getOrigin().y();
                    cam->translation[2] = rgbd_initial_poses[i].getOrigin().z();
                    cam->quaternion[0]  = rgbd_initial_poses[i].getRotation().w();
                    cam->quaternion[1]  = rgbd_initial_poses[i].getRotation().x();
                    cam->quaternion[2]  = rgbd_initial_poses[i].getRotation().y();
                    cam->quaternion[3]  = rgbd_initial_poses[i].getRotation().z();
                }

                reg_cameras.push_back(cam);
                problem.AddParameterBlock(cam->quaternion,4);
                problem.AddParameterBlock(cam->translation,3);

                number_of_constraints.push_back(0);
            }

            // cost functions
            vector<CostFunction*> rosrallCFs;
            vector<LossFunction*> allLFs;
            int total_constraints = 0;

            for (auto constr : rgbd_view_constraints_and_correspondences){
                Camera* cam1 = reg_cameras[constr.image1];
                Camera* cam2 = reg_cameras[constr.image2];

                if (constr.correspondences.size() < min_correspondences){
                    continue;
                }

                for (size_t k=0; k<constr.correspondences.size(); k++){
                    auto corresp = constr.correspondences[k];
                    auto corresp_2d = constr.correspondences_2d[k];
                    double point_original1[3] = {corresp.first.x, corresp.first.y, corresp.first.z};
                    double point_original2[3] = {corresp.second.x, corresp.second.y, corresp.second.z};

                    double point_original1_2d[2] = {corresp_2d.first.x, corresp_2d.first.y};
                    double point_original2_2d[2] = {corresp_2d.second.x, corresp_2d.second.y};
                    double depth1 = corresp.first.z;
                    double depth2 = corresp.second.z;

                    double weight = (corresp.first.getVector3fMap().squaredNorm() + corresp.second.getVector3fMap().squaredNorm())/2;

                    ceres::CostFunction *cost_function_proj = ProjectionResidual::Create(point_original1, point_original2, weight);
                    ceres::LossFunction *loss_function_proj = new ceres::HuberLoss(weight * 0.005);
                    problem.AddResidualBlock(cost_function_proj,
                                             loss_function_proj,
                                             cam1->quaternion,
                                             cam1->translation,
                                             cam2->quaternion,
                                             cam2->translation);
                    total_constraints++;
                }
                number_of_constraints[constr.image1] += constr.correspondences.size();
                number_of_constraints[constr.image2] += constr.correspondences.size();
            }

            if (!reg_cameras.size()){
                ROS_WARN_STREAM("RGBDViewRegistrationOptimizer ---- WARNING - no cameras defined for this object");
            }

            problem.SetParameterBlockConstant(reg_cameras[0]->quaternion);
            problem.SetParameterBlockConstant(reg_cameras[0]->translation);

//            ROS_INFO_STREAM("RGBD view optimization step 1: registering RGBD views. Num constratints "<<total_constraints);
//            Solver::Summary summary;
//            Solve(options, &problem, &summary);
//            if (m_bVerbose){
//                ROS_INFO_STREAM(summary.FullReport() << "\n");
//            }

//            ROS_INFO_STREAM("RGBD view optimization step 1: FINISHED. ");


//            // Registration to reference
//            Problem problem_reference;

//            // add cameras
//            for (int i=0; i<reg_cameras.size(); ++i){
//                problem_reference.AddParameterBlock(reg_cameras[i]->quaternion,4);
//                problem_reference.AddParameterBlock(reg_cameras[i]->translation,3);
//                problem_reference.SetParameterBlockConstant(reg_cameras[i]->quaternion);
//                problem_reference.SetParameterBlockConstant(reg_cameras[i]->translation);
//            }

//            Camera* transform_to_reference = new Camera();
//            problem_reference.AddParameterBlock(transform_to_reference->quaternion,4);
//            problem_reference.AddParameterBlock(transform_to_reference->translation,3);

            Camera* transform_to_reference = new Camera();
            problem.AddParameterBlock(transform_to_reference->quaternion,4);
            problem.AddParameterBlock(transform_to_reference->translation,3);

            for (auto constr : constraints_and_correspondences){
                Camera* cam1 = reg_cameras[constr.image1];
                Camera* cam2 = transform_to_reference;

                if (constr.correspondences.size() < min_correspondences){
                    continue;
                }

                for (size_t k=0; k<constr.correspondences.size(); k++){
                    auto corresp = constr.correspondences[k];
                    auto corresp_2d = constr.correspondences_2d[k];
                    double point_original1[3] = {corresp.first.x, corresp.first.y, corresp.first.z};
                    double point_original2[3] = {corresp.second.x, corresp.second.y, corresp.second.z};

                    double point_original1_2d[2] = {corresp_2d.first.x, corresp_2d.first.y};
                    double point_original2_2d[2] = {corresp_2d.second.x, corresp_2d.second.y};
                    double depth1 = corresp.first.z;
                    double depth2 = corresp.second.z;

                    double weight = (corresp.first.getVector3fMap().squaredNorm() + corresp.second.getVector3fMap().squaredNorm())/2;

                    ceres::CostFunction *cost_function_proj = ProjectionResidual::Create(point_original1, point_original2, weight);
                    ceres::LossFunction *loss_function_proj = new ceres::HuberLoss(weight * 0.005);
                    problem.AddResidualBlock(cost_function_proj,
                                             loss_function_proj,
                                             cam1->quaternion,
                                             cam1->translation,
                                             cam2->quaternion,
                                             cam2->translation);
                    total_constraints++;
                }
                number_of_constraints[constr.image1] += constr.correspondences.size();
            }


//            ROS_INFO_STREAM("RGBD view optimization step 2: registering to reference views. Num constratints "<<total_constraints);

//            Solver::Summary summary2;
//            Solve(options, &problem_reference, &summary2);
//            if (m_bVerbose){
//                ROS_INFO_STREAM(summary2.FullReport() << "\n");
//            }

//            ROS_INFO_STREAM("RGBD view optimization step 2: FINISHED."<<total_constraints);

            ROS_INFO_STREAM("RGBD view optimization with reference. Num constratints "<<total_constraints);
            Solver::Summary summary;
            Solve(options, &problem, &summary);
            if (m_bVerbose){
                ROS_INFO_STREAM(summary.FullReport() << "\n");
            }


            registered_poses.clear();

            tf::Quaternion tf_q(transform_to_reference->quaternion[1],transform_to_reference->quaternion[2],transform_to_reference->quaternion[3], transform_to_reference->quaternion[0]);
            tf::Vector3 tf_v(transform_to_reference->translation[0], transform_to_reference->translation[1], transform_to_reference->translation[2]);
            tf::Transform ceres_transform_to_reference(tf_q, tf_v);

            for (size_t i=0; i<reg_cameras.size();i++){
                tf::Quaternion tf_q(reg_cameras[i]->quaternion[1],reg_cameras[i]->quaternion[2],reg_cameras[i]->quaternion[3], reg_cameras[i]->quaternion[0]);
                tf::Vector3 tf_v(reg_cameras[i]->translation[0], reg_cameras[i]->translation[1], reg_cameras[i]->translation[2]);
                tf::Transform ceres_transform(tf_q, tf_v);
                ceres_transform = ceres_transform_to_reference.inverse() * ceres_transform;
                registered_poses.push_back(ceres_transform);
            }
            ROS_INFO_STREAM("RGBD view regitration with reference done ");

            // free memory
            // TODO check if this is necessary
            //        for (size_t i=0; i<allCFs.size();i++){
            //            delete allCFs[i];
            //        }
            //        for (size_t i=0; i<allLFs.size();i++){
            //            delete allLFs[i];
            //        }

            return true;
        }

    template<class PointType>
    std::vector<std::pair<PointType, PointType>>
    validateMatches(const std::vector<std::pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>>& matches,
                    const cv::Mat& image1, const cv::Mat& image2,
                    const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud1, const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud2,
                    const double& depth_threshold,
                    std::vector<std::pair<cv::Point2d, cv::Point2d>>& remaining_image_matches){
        using namespace std;

        std::vector<std::pair<PointType, PointType>> filtered_matches;

        // intermediate data structure
        pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences);
        // filter nans and depth outside range
        for (size_t i=0; i<matches.size(); i++){
            auto keypoint_pair = matches[i];
            // get point indices in point clouds
            long point_index1 = (int)keypoint_pair.first.y * image1.cols + (int)keypoint_pair.first.x;
            long point_index2 = (int)keypoint_pair.second.y * image2.cols + (int)keypoint_pair.second.x;
            PointType p1 = cloud1->points[point_index1];
            PointType p2 = cloud2->points[point_index2];

            if (!pcl::isFinite(p1) || !pcl::isFinite(p2)){
                continue; // skip nan matches
            }

            if (depth_threshold > 0.0){
                if ((p1.z > depth_threshold) || (p2.z > depth_threshold)){
                    continue; // skip points with depth outside desired range
                }
            }

            pcl::Correspondence c;
            c.index_match = point_index2;
            c.index_query = point_index1;
            c.distance = 1.0;
            correspondences_sift->push_back(c);
        }

        // next do RANSAC filtering
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> cr;
        pcl::Correspondences sac_correspondences;
        cr.setInputSource(cloud1);
        cr.setInputTarget(cloud2);
        cr.setInputCorrespondences(correspondences_sift);
        cr.setInlierThreshold(0.0125);
        cr.setMaximumIterations(10000);
        cr.getCorrespondences(sac_correspondences);

        if (sac_correspondences.size() == correspondences_sift->size()){ // could not find a consensus
            return filtered_matches;
        }

        for (auto corresp : sac_correspondences){
            pair<PointType, PointType> filtered_match(cloud1->points[corresp.index_query], cloud2->points[corresp.index_match]);
            filtered_matches.push_back(filtered_match);

            cv::Point2d p1_2d, p2_2d;
            p1_2d.y =  (corresp.index_query / image1.cols);
            p1_2d.x =  (corresp.index_query % image1.cols);
            p2_2d.y =  (corresp.index_match / image2.cols);
            p2_2d.x =  (corresp.index_match % image2.cols);
            pair<cv::Point2d, cv::Point2d> filtered_match_2d(p1_2d, p2_2d);
            remaining_image_matches.push_back(filtered_match_2d);
        }

        return filtered_matches;
    }

    template <class PointType>
    static std::pair<cv::Mat, cv::Mat> createRGBandDepthFromCloud(boost::shared_ptr<pcl::PointCloud<PointType>>& cloud)
    {
        std::pair<cv::Mat, cv::Mat> toRet;
        toRet.first = cv::Mat::zeros(480, 640, CV_8UC3); // RGB image
        toRet.second = cv::Mat::zeros(480, 640, CV_16UC1); // Depth image
        pcl::PointXYZRGB point;
        for (size_t y = 0; y < toRet.first.rows; ++y) {
            for (size_t x = 0; x < toRet.first.cols; ++x) {
                point = cloud->points[y*toRet.first.cols + x];
                // RGB
                toRet.first.at<cv::Vec3b>(y, x)[0] = point.b;
                toRet.first.at<cv::Vec3b>(y, x)[1] = point.g;
                toRet.first.at<cv::Vec3b>(y, x)[2] = point.r;
                // Depth
                if (!pcl::isFinite(point)) {
                    toRet.second.at<u_int16_t>(y, x) = 0.0; // convert to uint 16 from meters
                } else {
                    toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters
                }
            }
        }
        return toRet;
    }

private:
    bool m_bVerbose;
};


#endif
