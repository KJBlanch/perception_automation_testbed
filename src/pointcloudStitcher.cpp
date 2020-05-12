#include "pointcloudStitcher.h"

  void AlignmentRepresentation::param_setup () {

	ros::NodeHandle pcls_init;
	std::cout << "Starting Param setup" << std::endl;
	// Voxelisation
	if (pcls_init.getParam("/perception/parameters/perception/downsample", this->downsample)) {
		std::cout << "Downsample val found" << std::endl;
	} else {
		std::cout << "No Downsample val located. Defaulting.." << std::endl;
		this->downsample = true;
	}

	if (pcls_init.getParam("/perception/parameters/perception/voxel_size", this->voxel_size)) {
		std::cout << "Voxel size val found" << std::endl;
	} else {
		std::cout << "No Voxel size val located. Defaulting.." << std::endl;
		this->voxel_size = 0.1;
	}

	if (pcls_init.getParam("/perception/parameters/perception/kSearch", this->kSearch)) {
		std::cout << "kSearch val found" << std::endl;
	} else {
		std::cout << "No kSearch val located. Defaulting.." << std::endl;
		this->kSearch = 30;
	}

	if (pcls_init.getParam("/perception/parameters/perception/maxCorrespondenceDistance", this->maxCorrespondenceDistance)) {
		std::cout << "maxCorrespondenceDistance val found" << std::endl;
	} else {
		std::cout << "No maxCorrespondenceDistance val located. Defaulting.." << std::endl;
		this->maxCorrespondenceDistance = 0.2;
	}

	if (pcls_init.getParam("/perception/parameters/perception/iterations", this->iterations)) {
		std::cout << "iterations val found" << std::endl;
	} else {
		std::cout << "No iterations val located. Defaulting.." << std::endl;
		this->iterations = 50;
	}

	
	if (pcls_init.getParam("/perception/parameters/perception/transformationEpsilon", this->transformationEpsilon)) {
		std::cout << "transformationEpsilon val found" << std::endl;
	} else {
		std::cout << "No transformationEpsilon val located. Defaulting.." << std::endl;
		this->transformationEpsilon = 1e-6;
	}
	std::cout << "Finished Param setup" << std::endl;

  }

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void AlignmentRepresentation::pairAlign(PointCloud cloud_src, PointCloud cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample){
	// Downsample for consistency and speed
	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	const PointCloud::Ptr src_temp (new PointCloud);
	*src_temp = cloud_src;
	const PointCloud::Ptr tgt_temp (new PointCloud);
	*tgt_temp = cloud_tgt;


	if (downsample){
		pcl::VoxelGrid<PointT> grid;
		grid.setLeafSize(this->voxel_size, this->voxel_size, this->voxel_size);
		grid.setInputCloud(src_temp);
		grid.filter(*src);

		grid.setInputCloud(tgt_temp);
		grid.filter(*tgt);
	}
	else {
		*src = cloud_src;
		*tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(this->kSearch);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	// Weight the 'curvature' dimension so that it is balanced against x, y, and z
	AlignmentRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues(alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;

	reg.setTransformationEpsilon(this->transformationEpsilon);
	reg.setMaxCorrespondenceDistance(this->maxCorrespondenceDistance);  
	reg.setPointRepresentation(boost::make_shared<const AlignmentRepresentation>(point_representation));
	reg.setMaximumIterations(2);

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);


	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

	for (int i = 0; i < this->iterations; ++i)
	{
		//PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

			//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
		if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
		
		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	// Transform target back in source frame
	pcl::transformPointCloud(cloud_tgt, *output, targetToSource);

	*output += cloud_src;

	final_transform = targetToSource;
 }


Eigen::Matrix4f AlignmentRepresentation::stitcher_trigger(PointCloud source, PointCloud target){
	
	PointCloud::Ptr result(new PointCloud);
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

	PointCloud::Ptr temp(new PointCloud);
	AlignmentRepresentation::pairAlign(source, target, temp, pairTransform, this->downsample);
	

	//transform current pair into the global transform
	pcl::transformPointCloud(*temp, *result, pairTransform);
	
	if (this->save) {
		std::stringstream ss, ss2;
		ss << "source" << ".pcd";
		pcl::io::savePCDFile (ss.str (), source, true);
		ss2 << "target" << ".pcd";
		pcl::io::savePCDFile (ss2.str (), target, true);
	}
	return(pairTransform);
}