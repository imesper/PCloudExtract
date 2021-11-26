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

  program.add_argument("-f", "--input-folder")
      .help("specify the input folder.");
  program.add_argument("-o", "--output-file")
      .help("specify the output folder.")
      .default_value(std::string("output.csv"));
  program.add_argument("-F", "--output-folder")
      .help("specify the output file.")
      .default_value(std::string("output"));

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }
  if (program.is_used("--input-file") || program.is_used("--input-folder")) {
    std::cout << program;
  }
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
    files.emplace_back(inputFile);
  }
  if (processFolder) {
    for (auto &file : files) {
      BagFile bag(file);
      auto cloud = bag.getPointCloud(10);
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor(0.0, 0.0, 0.0);
      viewer.addPointCloud(cloud, "cloud");
      viewer.spin();
    }
  }

  return 0;
}
