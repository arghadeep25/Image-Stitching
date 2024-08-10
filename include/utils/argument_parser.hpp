/**
 * @file argument_parser.hpp
 * @details Header to parse the command line arguments
 * @author Arghadeep Mazumder
 * @version 0.1.0
 * @copyright -
 */
#ifndef IMAGE_STITCHING_ARGUMENT_PARSER_HPP_
#define IMAGE_STITCHING_ARGUMENT_PARSER_HPP_

#include <iostream>
#include <unistd.h>

namespace is::utils {
/**
 * @brief Parse the command line arguments.
 * @param argc The number of arguments.
 * @param argv The arguments.
 * @param input_path The input path.
 * @param output_path The output path.
 */
void parse(int argc, char *argv[], std::string &input_path, std::string &output_path) {
  if (argc < 5) {
    std::cerr << "Usage: " << argv[0] << " -i <input_path> -o <output_path>" << std::endl;
    exit(EXIT_FAILURE);
  }
  int opt;
  while ((opt = getopt(argc, argv, "i:o:")) != -1) {
    switch (opt) {
      case 'i':input_path = optarg;
        break;
      case 'o':output_path = optarg;
        break;
      default:std::cerr << "Usage: " << argv[0] << " -i <input_path> -o <output_path>" << std::endl;
        exit(EXIT_FAILURE);
    }
  }
}

} // namespace is::utils
#endif //IMAGE_STITCHING_ARGUMENT_PARSER_HPP_
