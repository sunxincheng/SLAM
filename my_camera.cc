#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    using clock = chrono::steady_clock;
    clock::time_point time_start_ = clock::now();
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    cv::Mat img;
    int count_=0;
    for(;;)
    {
      std::stringstream ss;
      ss << std::setw(6) << std::setfill('0') << count_ << ".png";
      string filename;
      ss >> filename;
      filename=argv[3]+filename;
     // ss.clear();
     // ss.str("");
      img=cv::imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
      if(img.empty())
	continue;
      double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      clock::now() - time_start_).count();
      SLAM.TrackMonocular(img,elapsed);
      count_++;
    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

}
