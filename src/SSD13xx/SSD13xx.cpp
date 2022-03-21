#include "SSD13xx.h"
#include "AbstractPlatform.h"

const ImgBufferFormat _get_img_fmt_for_ssd(const SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return ImgBufferFormat::MONOCHROME;
    case SSDModel::SSD1309:   return ImgBufferFormat::MONOCHROME;
    case SSDModel::SSD1331:   return ImgBufferFormat::R5_G6_B5;
    case SSDModel::SSD1351:   return ImgBufferFormat::R5_G6_B5;
  }
  return ImgBufferFormat::MONOCHROME;
}

const uint32_t _get_img_x_for_ssd(const SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return 128;
    case SSDModel::SSD1309:   return 128;
    case SSDModel::SSD1331:   return 96;
    case SSDModel::SSD1351:   return 128;
  }
  return 0;
}

const uint32_t _get_img_y_for_ssd(const SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return 64;
    case SSDModel::SSD1309:   return 64;
    case SSDModel::SSD1331:   return 64;
    case SSDModel::SSD1351:   return 128;
  }
  return 0;
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor.
*/
SSD13xx::SSD13xx(const SSD13xxOpts* O, const SSDModel M) :
  Image(
    _get_img_x_for_ssd(M),
    _get_img_y_for_ssd(M),
    _get_img_fmt_for_ssd(M)
  ),
  _opts(O), _model(M), _flags(0)
{
  _is_framebuffer(true);
}
