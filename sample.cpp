#include "sample.h"

Sample::Sample(double vg1_, double va_, double ia_, double vg2_, double ig2_, double vh_, double ih_) : vg1(vg1_), va(va_), ia(ia_), vg2(vg2_), ig2(ig2_), vh(vh_), ih(ih_)
{

}

double Sample::getVa() const
{
    return va;
}

double Sample::getVg1() const
{
    return vg1;
}

double Sample::getVg2() const
{
    return vg2;
}

double Sample::getIa() const
{
    return ia;
}

double Sample::getIg2() const
{
    return ig2;
}

double Sample::getVh() const
{
    return vh;
}

double Sample::getIh() const
{
    return ih;
}
