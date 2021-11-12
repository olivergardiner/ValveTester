#ifndef SAMPLE_H
#define SAMPLE_H


class Sample
{
public:
    Sample(double va_, double vg1_, double vg2_, double ia_, double ig2_, double vh_, double ih_);

    double getVa() const;
    double getVg1() const;
    double getVg2() const;
    double getIa() const;
    double getIg2() const;
    double getVh() const;
    double getIh() const;

private:
    double vg1;
    double va;
    double ia;
    double vg2;
    double ig2;
    double vh;
    double ih;
};

#endif // SAMPLE_H
