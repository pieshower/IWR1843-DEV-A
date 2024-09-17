#include "../include/mmWaveRadar_imp.h"

data_header_t dataHeader;
data_tl_t dataTL;
detected_object_t detectedObject;
std::vector<detected_object_t> detectedObjects;
data_complete_t dataComplete;

std::mutex mtx;