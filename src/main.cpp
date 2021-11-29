#include "BagFile.h"
#include "argparse.h"

#include <filesystem>
#include <iostream>
#include <pybind11/embed.h>

namespace py = pybind11;

int main(int argc, char *argv[]) {

  py::scoped_interpreter guard{};
  argparse::ArgumentParser program("PCloudExtract", "0.0.1");

  program.add_argument("-i", "--input-file").help("specify the input file.");
  program.add_argument("-I", "--input-folder")
      .help("specify the input folder.");
  program.add_argument("-o", "--output-file")
      .help("specify the output folder.")
      .default_value(std::string("output.csv"));
  program.add_argument("-O", "--output-folder")
      .help("specify the output file.")
      .default_value(std::string("../output"));
  program.add_argument("-f", "--filter")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("-x", "--x-filter")
      .help("specify x.")
      .nargs(2)
      .default_value(std::vector<float>{-0.5, 1.0})
      .scan<'g', float>();
  program.add_argument("-z", "--z-filter")
      .help("specify z.")
      .nargs(2)
      .default_value(std::vector<float>{0.5, 1.0})
      .scan<'g', float>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }
  if (!program.is_used("--input-file") && !program.is_used("--input-folder")) {
    std::cout << program;
    exit(-1);
  }
  auto filter = program.get<bool>("--filter");
  std::cout << filter << std::endl;
  auto filterX = program.get<std::vector<float>>("--x-filter");
  auto filterZ = program.get<std::vector<float>>("--z-filter");

  std::string inputFile, inputFolder, outputFile, outputFolder;
  if (auto input = program.present<std::string>("--input-file"))
    inputFile = *input;
  if (auto input = program.present<std::string>("--input-folder"))
    inputFolder = *input;

  outputFile = program.get<std::string>("--output-file");
  outputFolder = program.get<std::string>("--output-folder");

  std::vector<std::string> files;

  bool processFolder = false;
  if (!inputFolder.empty()) {
    processFolder = true;
    for (auto &file : std::filesystem::directory_iterator(inputFolder)) {
      std::cout << file.path() << std::endl;
      if (file.is_regular_file())
        files.emplace_back(file.path());
    }
  } else if (!inputFile.empty()) {
    std::cout << "Input File: " << inputFile << std::endl;
    files.emplace_back(inputFile);
  }
  for (auto &file : files) {
    std::cout << file << std::endl;
    std::string filename =
        outputFolder + file.substr(file.find_last_of("/"), file.find(".bag")) +
        ".txt";
    std::cout << filename << std::endl;
    bool flipped = file.find("YES") != -1 ? true : false;
    BagFile bag(file);
    auto cloud = bag.getPointCloud(10, flipped, filter, filterX, filterZ);
    ////    pcl::PointXYZRGB point1, point2;
    ////    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, point1, point2);
    ////    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    ////    viewer.addCoordinateSystem(0.2);
    ////    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    ////    viewer.addPointCloud(cloud, "cloud");
    ////    viewer.addSphere(point1, 0.02, "1");
    ////    viewer.addSphere(point2, 0.02, "2");
    ////    viewer.spinOnce();
    ////    viewer.close();
    Helper::saveFileTXT<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(filename,
                                                                cloud);
  }
  std::cout << files.size() << std::endl;
  return 0;
}
