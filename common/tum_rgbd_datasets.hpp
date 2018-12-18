#ifndef TUM_RGBD_DATASETS
#define TUM_RGBD_DATASETS

enum DatasetName{
	// Test and Debug
	fr1_rpy,
	fr1_xyz,
	fr2_rpy,
	fr2_xyz,
	// Handheld SLAM
	fr1_360,
	fr1_floor,
	fr1_desk,
	fr1_desk2,
	fr1_room,
	fr2_360_hemisphere,
	// kidnap
	fr2_desk,
	fr2_large_no_loop,
	fr2_large_with_loop,
	fr3_long_office_household,
	// Robot SLAM
	fr2_pioneer_360,
	fr2_pioneer_slam,
	fr2_pioneer_slam2,
	fr2_pioneer_slam3,
	// Structure vs. Texture
	fr3_nostructure_notexture_far,
	fr3_nostructure_notexture_near_withloop,
	fr3_nostructure_texture_far,
	fr3_nostructure_texture_near_withloop,
	fr3_structure_notexture_far,
	fr3_structure_notexture_near,
	fr3_structure_texture_far,
	fr3_structure_texture_near,
	//Dynamic Objects
	fr2_desk_with_person,
	fr3_sitting_static,
	fr3_sitting_xyz,
	fr3_sitting_halfsphere,
	fr3_sitting_rpy,
	fr3_walking_static,
	fr3_walking_xyz,
	fr3_walking_halfsphere,
	fr3_walking_rpy,
	NoDatasets};

// GROUNDTURTHS
// ------------

std::vector<const char*> groundtruths{"data/groundtruth/groundtruth_fr01_rpy.txt", // Testing and Debugging Datasets
                         "data/groundtruth/groundtruth_fr01_xyz.txt",
                         "data/groundtruth/groundtruth_fr02_rpy.txt",
                         "data/groundtruth/groundtruth_fr02_xyz.txt",
                         "data/groundtruth/groundtruth_fr11_360.txt", // Handheld SLAM
                         "data/groundtruth/groundtruth_fr11_floor.txt",
                         "data/groundtruth/groundtruth_fr11_desk.txt",
                         "data/groundtruth/groundtruth_fr11_desk2.txt",
                         "data/groundtruth/groundtruth_fr11_room.txt",
                         "data/groundtruth/groundtruth_fr12_360_hemisphere.txt",
                         //"data/groundtruth/groundtruth_fr12_360_kidnap.txt", // Este no se usa porque no es una secuencia continua
                         "data/groundtruth/groundtruth_fr12_desk.txt",
                         "data/groundtruth/groundtruth_fr12_large_no_loop.txt",
                         "data/groundtruth/groundtruth_fr12_large_with_loop.txt",
                         "data/groundtruth/groundtruth_fr13_long_office_household.txt",
                         "data/groundtruth/groundtruth_fr22_pioneer_360.txt", // Robot SLAM
                         "data/groundtruth/groundtruth_fr22_pioneer_slam.txt",
                         "data/groundtruth/groundtruth_fr22_pioneer_slam2.txt",
                         "data/groundtruth/groundtruth_fr22_pioneer_slam3.txt",
                         "data/groundtruth/groundtruth_fr33_nostructure_notexture_far.txt", // Structure vs. Texture
                         "data/groundtruth/groundtruth_fr33_nostructure_notexture_near_withloop.txt",
                         "data/groundtruth/groundtruth_fr33_nostructure_texture_far.txt",
                         "data/groundtruth/groundtruth_fr33_nostructure_texture_near_withloop.txt",
                         "data/groundtruth/groundtruth_fr33_structure_notexture_far.txt",
                         "data/groundtruth/groundtruth_fr33_structure_notexture_near.txt",
                         "data/groundtruth/groundtruth_fr33_structure_texture_far.txt",
                         "data/groundtruth/groundtruth_fr33_structure_texture_near.txt",
                         "data/groundtruth/groundtruth_fr42_desk_with_person.txt", // Dynamic Objects
                         "data/groundtruth/groundtruth_fr43_sitting_static.txt",
                         "data/groundtruth/groundtruth_fr43_sitting_xyz.txt",
                         "data/groundtruth/groundtruth_fr43_sitting_halfsphere.txt",
                         "data/groundtruth/groundtruth_fr43_sitting_rpy.txt",
                         "data/groundtruth/groundtruth_fr43_walking_static.txt",
                         "data/groundtruth/groundtruth_fr43_walking_xyz.txt",
                         "data/groundtruth/groundtruth_fr43_walking_halfsphere.txt",
                         "data/groundtruth/groundtruth_fr43_walking_rpy.txt"};

// OUT_FAST2
// ------------

std::vector<const char*> out_fast2{"data/out_fast2/out_fast2_freiburg01_rpy.txt", // Testing and Debugging Datasets
                         "data/out_fast2/out_fast2_freiburg01_xyz.txt",
                         "data/out_fast2/out_fast2_freiburg02_rpy.txt",
                         "data/out_fast2/out_fast2_freiburg02_xyz.txt",
                         "data/out_fast2/out_fast2_freiburg11_360.txt", // Handheld SLAM
                         "data/out_fast2/out_fast2_freiburg11_floor.txt",
                         "data/out_fast2/out_fast2_freiburg11_desk.txt",
                         "data/out_fast2/out_fast2_freiburg11_desk2.txt",
                         "data/out_fast2/out_fast2_freiburg11_room.txt",
                         "data/out_fast2/out_fast2_freiburg12_360_hemisphere.txt",
                         //"data/out_fast2/out_fast2_freiburg12_360_kidnap.txt", // Este no se usa porque no es una secuencia continua
                         "data/out_fast2/out_fast2_freiburg12_desk.txt",
                         "data/out_fast2/out_fast2_freiburg12_large_no_loop.txt",
                         "data/out_fast2/out_fast2_freiburg12_large_with_loop.txt",
                         "data/out_fast2/out_fast2_freiburg13_long_office_household.txt",
                         "data/out_fast2/out_fast2_freiburg22_pioneer_360.txt", // Robot SLAM
                         "data/out_fast2/out_fast2_freiburg22_pioneer_slam.txt",
                         "data/out_fast2/out_fast2_freiburg22_pioneer_slam2.txt",
                         "data/out_fast2/out_fast2_freiburg22_pioneer_slam3.txt",
                         "data/out_fast2/out_fast2_freiburg33_nostructure_notexture_far.txt", // Structure vs. Texture
                         "data/out_fast2/out_fast2_freiburg33_nostructure_notexture_near_withloop.txt",
                         "data/out_fast2/out_fast2_freiburg33_nostructure_texture_far.txt",
                         "data/out_fast2/out_fast2_freiburg33_nostructure_texture_near_withloop.txt",
                         "data/out_fast2/out_fast2_freiburg33_structure_notexture_far.txt",
                         "data/out_fast2/out_fast2_freiburg33_structure_notexture_near.txt",
                         "data/out_fast2/out_fast2_freiburg33_structure_texture_far.txt",
                         "data/out_fast2/out_fast2_freiburg33_structure_texture_near.txt",
                         "data/out_fast2/out_fast2_freiburg42_desk_with_person.txt", // Dynamic Objects
                         "data/out_fast2/out_fast2_freiburg43_sitting_static.txt",
                         "data/out_fast2/out_fast2_freiburg43_sitting_xyz.txt",
                         "data/out_fast2/out_fast2_freiburg43_sitting_halfsphere.txt",
                         "data/out_fast2/out_fast2_freiburg43_sitting_rpy.txt",
                         "data/out_fast2/out_fast2_freiburg43_walking_static.txt",
                         "data/out_fast2/out_fast2_freiburg43_walking_xyz.txt",
                         "data/out_fast2/out_fast2_freiburg43_walking_halfsphere.txt",
                         "data/out_fast2/out_fast2_freiburg43_walking_rpy.txt"};

std::vector<const char*> out_fast{"data/out_fast/out_fast_freiburg01_rpy.txt", // Testing and Debugging Datasets
                         "data/out_fast/out_fast_freiburg01_xyz.txt",
                         "data/out_fast/out_fast_freiburg02_rpy.txt",
                         "data/out_fast/out_fast_freiburg02_xyz.txt",
                         "data/out_fast/out_fast_freiburg11_360.txt", // Handheld SLAM
                         "data/out_fast/out_fast_freiburg11_floor.txt",
                         "data/out_fast/out_fast_freiburg11_desk.txt",
                         "data/out_fast/out_fast_freiburg11_desk2.txt",
                         "data/out_fast/out_fast_freiburg11_room.txt",
                         "data/out_fast/out_fast_freiburg12_360_hemisphere.txt",
                         //"data/out_fast/out_fast_freiburg12_360_kidnap.txt", // Este no se usa porque no es una secuencia continua
                         "data/out_fast/out_fast_freiburg12_desk.txt",
                         "data/out_fast/out_fast_freiburg12_large_no_loop.txt",
                         "data/out_fast/out_fast_freiburg12_large_with_loop.txt",
                         "data/out_fast/out_fast_freiburg13_long_office_household.txt",
                         "data/out_fast/out_fast_freiburg22_pioneer_360.txt", // Robot SLAM
                         "data/out_fast/out_fast_freiburg22_pioneer_slam.txt",
                         "data/out_fast/out_fast_freiburg22_pioneer_slam2.txt",
                         "data/out_fast/out_fast_freiburg22_pioneer_slam3.txt",
                         "data/out_fast/out_fast_freiburg33_nostructure_notexture_far.txt", // Structure vs. Texture
                         "data/out_fast/out_fast_freiburg33_nostructure_notexture_near_withloop.txt",
                         "data/out_fast/out_fast_freiburg33_nostructure_texture_far.txt",
                         "data/out_fast/out_fast_freiburg33_nostructure_texture_near_withloop.txt",
                         "data/out_fast/out_fast_freiburg33_structure_notexture_far.txt",
                         "data/out_fast/out_fast_freiburg33_structure_notexture_near.txt",
                         "data/out_fast/out_fast_freiburg33_structure_texture_far.txt",
                         "data/out_fast/out_fast_freiburg33_structure_texture_near.txt",
                         "data/out_fast/out_fast_freiburg42_desk_with_person.txt", // Dynamic Objects
                         "data/out_fast/out_fast_freiburg43_sitting_static.txt",
                         "data/out_fast/out_fast_freiburg43_sitting_xyz.txt",
                         "data/out_fast/out_fast_freiburg43_sitting_halfsphere.txt",
                         "data/out_fast/out_fast_freiburg43_sitting_rpy.txt",
                         "data/out_fast/out_fast_freiburg43_walking_static.txt",
                         "data/out_fast/out_fast_freiburg43_walking_xyz.txt",
                         "data/out_fast/out_fast_freiburg43_walking_halfsphere.txt",
                         "data/out_fast/out_fast_freiburg43_walking_rpy.txt"};




#endif // TUM_RGBD_DATASETS