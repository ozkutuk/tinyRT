#ifndef PPMIMAGE_H
#define PPMIMAGE_H

#include <vector>
#include <string>

struct PPMColor {
    int r;
    int g;
    int b;
};

class PPMImage {
private:
    int width = 0;
    int height = 0;
    int max_val;
    std::vector<PPMColor> raster;

public:
    //PPMImage();
    PPMImage(int width, int height, int max_val = 255);
    ~PPMImage() = default;
    void setPixel(int x, int y, PPMColor color);
    void writeFile(const std::string &filename);
};

#endif
