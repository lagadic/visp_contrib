//! \example tutorial-contrast-sharpening.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if defined(VISP_HAVE_MODULE_IMGPROC)
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

int main(int argc, const char ** argv) {
  //! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
  //!
  std::string input_filename = "Crayfish-low-contrast.ppm";
  unsigned int size = 7;
  double weight = 0.6;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i+1 < argc) {
      input_filename = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--size" && i+1 < argc) {
      size = (unsigned int) atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--weight" && i+1 < argc) {
      weight = atof(argv[i+1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <input image>] [--size <Gaussian kernel size>]"
                   " [--weight <unsharp mask weighting>] [--help]"
                << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Read]
  vpImage<vpRGBa> I_color;
  vpImageIo::read(I_color, input_filename);
  //! [Read]

#ifdef VISP_HAVE_X11
  vpDisplayX d, d2, d3, d4, d5;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d, d2, d3, d4;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d, d2, d3, d4;
#endif
  d.init(I_color, 0, 0, "Input color image");

  //! [Stretch contrast]
  vpImage<vpRGBa> I_stretch;
  vp::stretchContrast(I_color, I_stretch);
  //! [Stretch contrast]
  d2.init(I_stretch, I_color.getWidth(), 10, "Stretch contrast");

  //! [Stretch contrast HSV]
  vpImage<vpRGBa> I_stretch_hsv;
  vp::stretchContrastHSV(I_color, I_stretch_hsv);
  //! [Stretch contrast HSV]
  d3.init(I_stretch_hsv, 0, I_color.getHeight()+80, "Stretch contrast HSV");

  //! [Histogram equalization]
  vpImage<vpRGBa> I_hist_eq;
  vp::equalizeHistogram(I_color, I_hist_eq);
  //! [Histogram equalization]
  d4.init(I_hist_eq, I_color.getWidth(), I_color.getHeight()+80, "Histogram equalization");

  //! [Unsharp mask]
  vpImage<vpRGBa> I_unsharp;
  vp::unsharpMask(I_stretch_hsv, I_unsharp, size, weight);
  //! [Unsharp mask]
  d5.init(I_unsharp, 0, 2*I_color.getHeight()+80, "Unsharp mask");

  vpDisplay::display(I_color);
  vpDisplay::display(I_stretch);
  vpDisplay::display(I_stretch_hsv);
  vpDisplay::display(I_hist_eq);
  vpDisplay::display(I_unsharp);
  vpDisplay::displayText(I_unsharp, 20, 20, "Click to quit.", vpColor::red);
  vpDisplay::flush(I_color);
  vpDisplay::flush(I_stretch);
  vpDisplay::flush(I_stretch_hsv);
  vpDisplay::flush(I_hist_eq);
  vpDisplay::flush(I_unsharp);
  vpDisplay::getClick(I_unsharp);

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
  return 0;
#endif
}
