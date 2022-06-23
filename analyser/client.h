#pragma once


class Client
{
public:
    Client();

    virtual void updateHeater(double vh, double ih) = 0;
    virtual void testProgress(int progress) = 0;
    virtual void testFinished() = 0;
    virtual void testAborted() = 0;
};

