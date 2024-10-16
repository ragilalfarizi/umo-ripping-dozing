#include "gps.h"

GPS::GPS()
{
}

GPS::~GPS()
{
}

double GPS::getLongitude()
{
    return _gps.location.lng();
}

double GPS::getlatitude()
{
    return _gps.location.lat();
}

bool GPS::getValidation()
{
    return _gps.location.isValid();
}

int32_t GPS::getCharProcessed()
{
    return _gps.charsProcessed();
}

void GPS::encode(char c)
{
    _gps.encode(c);
}
