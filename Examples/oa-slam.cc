/**
 * This file is part of OA-SLAM.
 *
 * Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
 * (Inria, LORIA, Université de Lorraine)
 * OA-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OA-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdlib.h> /* srand, rand */

#include <opencv2/core/core.hpp>

#include "Osmap.h"
#include "Utils.h"
// #include "Utils.h"
#include <ImageDetections.h>
#include <System.h>
#include <experimental/filesystem>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace fs = std::experimental::filesystem;

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
  srand(time(nullptr));
  std::cout << "C++ version: " << __cplusplus << std::endl;

  if (argc != 8)
  {
    cerr << endl
         << "Usage:\n"
            " ./oa-slam\n"
            "      vocabulary_file\n"
            "      camera_file\n"
            "      path_to_image_sequence (.txt file listing the images or a "
            "folder with rgb.txt or 'webcam_id')\n"
            "      detections_file (.json file with detections or .onnx yolov5 "
            "weights)\n"
            "      categories_to_ignore_file (file containing the categories "
            "to ignore (one category_id per line))\n"
            "      relocalization_mode ('points', 'objects' or "
            "'points+objects')\n"
            "      output_name \n";
    return 1;
  }

  std::string vocabulary_file = string(argv[1]);
  std::string parameters_file = string(argv[2]);
  string path_to_images = string(argv[3]);
  std::string detections_file(argv[4]);
  std::string categories_to_ignore_file(argv[5]);
  string reloc_mode = string(argv[6]);
  string output_name = string(argv[7]);

  // possible to pass 'webcam_X' where 'X' is the webcam id
  bool use_webcam = false;
  bool use_video = false;
  int webcam_id = 0;
  std::cout << "Video file used: " << path_to_images << '\n';
  if (path_to_images.size() >= 6 && path_to_images.substr(0, 6) == "webcam")
  {
    use_webcam = true;
    if (path_to_images.size() > 7)
    {
      webcam_id = std::stoi(path_to_images.substr(7));
    }
  }

  if (path_to_images.size() >= 4 &&
      path_to_images.substr(path_to_images.size() - 4) == ".avi")
  {
    use_video = true;
    std::cout << "Video has .avi filetype. \n";
  }

  // Possible to pass a file listing images instead of a folder containing a
  // file rgb.txt or "webcam"
  std::string image_list_file = "rgb.txt";
  // int nn = path_to_images.size();
  if (!use_webcam && get_file_extension(path_to_images) == "txt")
  {
    int pos = path_to_images.find_last_of('/');
    image_list_file = path_to_images.substr(pos + 1);
    path_to_images = path_to_images.substr(0, pos + 1);
  }

  if (!use_webcam && path_to_images.back() != '/')
    path_to_images += "/";

  output_folder = output_name;
  if (output_folder.back() != '/')
    output_folder += "/";
  fs::create_directories(output_folder);

  // Load categories to ignore
  std::ifstream fin(categories_to_ignore_file);
  vector<int> classes_to_ignore;
  if (!fin.is_open())
  {
    std::cout << "Warning !! Failed to open the file with ignore classes. No "
                 "class will be ignore.\n";
  }
  else
  {
    int cat;
    while (fin >> cat)
    {
      std::cout << "Ignore category: " << cat << "\n";
      classes_to_ignore.push_back(cat);
    }
  }

  // Load object detections
  auto extension = get_file_extension(detections_file);
  std::shared_ptr<ORB_SLAM2::ImageDetectionsManager> detector = nullptr;
  bool detect_from_file = false;
  if (extension == "onnx")
  { // load network
    detector = std::make_shared<ORB_SLAM2::ObjectDetector>(detections_file,
                                                           classes_to_ignore);
    detect_from_file = false;
  }
  else if (extension == "json")
  { // load from external detections file
    detector = std::make_shared<ORB_SLAM2::DetectionsFromFile>(
        detections_file, classes_to_ignore);
    detect_from_file = true;
  }
  else
  {
    std::cout << "Invalid detection file. It should be .json or .onnx\n"
                 "No detections will be obtained.\n";
  }

  // Relocalization mode
  ORB_SLAM2::enumRelocalizationMode relocalization_mode =
      ORB_SLAM2::RELOC_POINTS;
  if (reloc_mode == string("points"))
    relocalization_mode = ORB_SLAM2::RELOC_POINTS;
  else if (reloc_mode == std::string("objects"))
    relocalization_mode = ORB_SLAM2::RELOC_OBJECTS;
  else if (reloc_mode == std::string("points+objects"))
    relocalization_mode = ORB_SLAM2::RELOC_OBJECTS_POINTS;
  else
  {
    std::cerr << "Error: Invalid parameter for relocalization mode. "
                 "It should be 'points', 'objects' or 'points+objects'.\n";
    return 1;
  }

  // Load images
  cv::VideoCapture cap;
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  int nImages = 10000;
  if (!use_webcam && !use_video)
  {
    string strFile = path_to_images + image_list_file;
    LoadImages(strFile, vstrImageFilenames, vTimestamps);
    nImages = vstrImageFilenames.size();
  }
  else if (use_video)
  {
    std::cout << "Opening video file!\n";
    path_to_images.resize(path_to_images.size() - 1);
    if (cap.open(path_to_images, cv::CAP_FFMPEG))
    {
      std::cout << "Opened video: " << path_to_images << '\n';
      // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }
    else
    {
      std::cerr << "Failed to open video: " << path_to_images << '\n';
      return -1;
    }
  }

  else
  {
    if (cap.open(webcam_id, cv::CAP_V4L2))
    {
      std::cout << "Opened webcam: " << webcam_id << "\n";
      cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }
    else
    {
      std::cerr << "Failed to open webcam: " << webcam_id << "\n";
      return -1;
    }
  }

  // Create system
  ORB_SLAM2::System SLAM(vocabulary_file, parameters_file,
                         ORB_SLAM2::System::MONOCULAR, true, true, false);
  SLAM.SetRelocalizationMode(relocalization_mode);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.reserve(nImages);

  std::cout << endl
            << "-------" << endl;
  std::cout << "Start processing sequence ..." << endl;
  std::cout << "Images in the sequence: " << nImages << endl
            << endl;

  ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);

  // Main loop
  cv::Mat im;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
  poses.reserve(nImages);
  std::vector<std::string> filenames;
  filenames.reserve(nImages);
  std::vector<double> timestamps;
  timestamps.reserve(nImages);
  int ni = 0;
  int idx_before_skip = ni;
  int frameToSkip = 0;

  int index_adjuster = 0;
  vector<double> low_1_percent_fps_history;
  double low_1_percent_fps = 0.00;

  // File logging for detection time
  std::ofstream detection_ms;
  detection_ms.open(output_folder + "detection_ms.csv");
  if (!detection_ms.is_open())
  {
    std::cerr << "Error creating/opening file\n";
  }
  else
  {
    detection_ms << "frame,duration_ms" << '\n';
  }

  std::ofstream total_execution_time;
  total_execution_time.open(output_folder + "total_execution_time.csv");
  if (!total_execution_time.is_open())
  {
    std::cerr << "Error creating/opening file\n";
  }
  else
  {
    total_execution_time << "frame,duration_s" << '\n';
  }
  if (use_video)
    cap >> im;

  bool wasPaused = 0;

  while (1)
  {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::string filename;
    if (use_webcam || use_video)
    {
      // int some = ni + frameToSkip;
      // cap >> im; // get image from webcam
      filename = "frame_" + std::to_string(ni) + ".png";
      // std::cout << ni << '\n';
    }
    else
    {
      filename = path_to_images + vstrImageFilenames[ni];
      im = cv::imread(filename, cv::IMREAD_UNCHANGED); // read image from disk
    }
    double tframe =
        ni < vTimestamps.size() ? vTimestamps[ni] : std::time(nullptr);
    timestamps.push_back(tframe);
    if (im.empty())
    {
      cerr << endl
           << "Failed to load image: " << filename << endl;
      return 1;
    }
    filenames.push_back(filename);

    auto clahe = cv::createCLAHE(1.5, cv::Size(2, 2));

    cv::Mat lab_im;
    std::vector<cv::Mat> lab_planes(3);
    cv::GaussianBlur(im, im, cv::Size(3, 3), 3.4);
    cv::cvtColor(im, lab_im, cv::COLOR_BGR2Lab);
    cv::split(lab_im, lab_planes);

    cv::Mat lab_channel;
    clahe->apply(lab_planes[0], lab_channel);

    lab_channel.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_im);

    cv::cvtColor(lab_im, im, cv::COLOR_Lab2BGR);
    cv::GaussianBlur(im, im, cv::Size(3, 3), 2.0);

    // Get object detections
    std::vector<ORB_SLAM2::Detection::Ptr> detections;
    if (detector)
    {
      if (detect_from_file)
        detections = detector->detect(filename); // from detections file
      else
      {
        Timer timer;
        timer.start();
        detections = detector->detect(im); // from neural network
        detection_ms << ni << ',' << timer.stop() << '\n';
      }
    }

    auto tbefore_track = std::chrono::steady_clock::now();

    // Pass the image and detections to the SLAM system
    cv::Mat m = SLAM.TrackMonocular(im, tframe, detections, false);

    if (m.rows && m.cols)
      poses.push_back(ORB_SLAM2::cvToEigenMatrix<double, float, 4, 4>(m));
    else
      poses.push_back(Eigen::Matrix4d::Identity());

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack;
    if (wasPaused)
    {
      ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(tbefore_track - t1)
                   .count();
      wasPaused = 0;
    }
    else
    {
      ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
          .count();
    }
    vTimesTrack.push_back(ttrack);
    // std::cout << "time = " << ttrack << "\n";
    std::cout << "FPS = " << 1 / ttrack << '\n';
    total_execution_time << ni << ',' << ttrack << '\n';

    // Calculate frame to skip if use video to simulate real time
    // Example: video recorded at 30 FPS, 1 frame took 0.1s to process, so the number of frames to skip is

    if (use_video)
    {
      if (!SLAM.mpViewer->isPaused())
      {
        std::cout << "processing time: " << ttrack << '\n';
        frameToSkip = round(30 * ttrack) - 1;
        for (int i = 0; i < frameToSkip; i++)
        {
          cap >> im;
        }
      }
      else
      {
        std::cout << "Video paused.\n";
        wasPaused = 1;
        frameToSkip = 0;
      }
    }

    if (SLAM.ShouldQuit())
      break;

    ++ni;

    if (ni >= nImages)
      break;
  }

  double sum_of_low_fps = std::accumulate(low_1_percent_fps_history.begin(),
                                          low_1_percent_fps_history.end(), 0.0);
  double average_low_1_percent_fps =
      sum_of_low_fps / low_1_percent_fps_history.size();

  // Stop all threads
  SLAM.Shutdown();
  detection_ms.close();
  total_execution_time.close();

  // Save camera tracjectory

  // TXT files
  std::ofstream file(output_folder + "camera_poses_" + output_name + ".txt");
  std::ofstream file_tum(output_folder + "camera_poses_" + output_name +
                         "_tum.txt"); // output poses in the TUM RGB-D format
  json json_data;
  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    Eigen::Matrix4d m = poses[i];
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
    pose.block<3, 1>(0, 3) =
        -m.block<3, 3>(0, 0).transpose() * m.block<3, 1>(0, 3);

    file << i << " " << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2)
         << " " << pose(0, 3) << " " << pose(1, 0) << " " << pose(1, 1) << " "
         << pose(1, 2) << " " << pose(1, 3) << " " << pose(2, 0) << " "
         << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";

    json R({{m(0, 0), m(0, 1), m(0, 2)},
            {m(1, 0), m(1, 1), m(1, 2)},
            {m(2, 0), m(2, 1), m(2, 2)}});
    json t({m(0, 3), m(1, 3), m(2, 3)});
    json image_data;
    image_data["file_name"] = filenames[i];
    image_data["R"] = R;
    image_data["t"] = t;
    json_data.push_back(image_data);

    auto q = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
    auto p = pose.block<3, 1>(0, 3);
    file_tum << std::fixed << timestamps[i] << " " << p[0] << " " << p[1] << " "
             << p[2] << " " << q.x() << " " << q.y() << " " << q.z() << " "
             << q.w() << "\n";
  }
  file.close();
  file_tum.close();

  // JSON files
  std::ofstream json_file(output_folder + "camera_poses_" + output_name +
                          ".json");
  json_file << json_data;
  json_file.close();

  // Tracking time statistics
  std::sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++)
  {
    totaltime += vTimesTrack[ni];
  }
  std::cout << "-------" << endl
            << endl;
  std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  std::cout << "mean tracking time: " << totaltime / nImages << endl
            << endl;
  std::cout << "average FPS: " << 1 / (totaltime / nImages) << '\n';
  std::cout << "average low 1% FPS: " << average_low_1_percent_fps << '\n';

  // Save camera trajectory, points and objects
  SLAM.SaveKeyFrameTrajectoryTUM(output_folder + "keyframes_poses_" +
                                 output_name + "_tum.txt");
  SLAM.SaveKeyFrameTrajectoryJSON(
      output_folder + "keyframes_poses_" + output_name + ".json", filenames);
  SLAM.SaveMapPointsOBJ(output_folder + "map_points_" + output_name + ".obj");
  SLAM.SaveMapObjectsOBJ(output_folder + "map_objects_" + output_name + ".obj");
  SLAM.SaveMapObjectsTXT(output_folder + "map_objects_" + output_name + ".txt");
  std::cout << "\n";

  // Save a reloadable map
  osmap.mapSave(output_folder + "map_" + output_name);

  FileManager::closeAll();

  return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps)
{
  ifstream f;
  f.open(strFile.c_str());

  string s0;
  double t = 0;
  int n = 0;
  bool found_timestamps = false;
  while (!f.eof())
  {
    string s;
    getline(f, s);
    if (!s.empty() && s[0] != '#')
    {
      stringstream ss;
      ss << s;
      string sRGB;
      if (ss.str().find(' ') != std::string::npos)
      {
        ss >> t;
        found_timestamps = true;
      }
      ss >> sRGB;

      vTimestamps.push_back(t);
      vstrImageFilenames.push_back(sRGB);
      if (!found_timestamps)
        t += 0.033;
      ++n;
    }
  }
  f.close();
}
