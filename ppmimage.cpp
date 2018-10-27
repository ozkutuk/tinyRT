#include "ppmimage.h"

#include <fstream>

PPMImage::PPMImage(int width, int height, int max_val)
    : width(width),
      height(height),
      max_val(max_val),
      raster(width * height) {

}

void PPMImage::setPixel(int x, int y, PPMColor color) {
    const int raster_index = y * width + x;
    raster[raster_index] = color;
}

void PPMImage::writeFile(const std::string &filename) {
    std::ofstream out_file;

    out_file.open(filename);
    out_file << "P3\n"; // magic
    out_file << "# " << filename << "\n"; // filename as comment
    out_file << width << " " << height << "\n";
    out_file << max_val << "\n";

    for (auto & color : raster)
        out_file << color.r << " "
                 << color.g << " "
                 << color.b << "\n";

    out_file.close();
}
