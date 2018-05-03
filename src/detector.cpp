// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


class Detector
{
    BufferedPort<ImageOf<PixelRgb> > imagePort;  // make a port for reading images
    BufferedPort<ImageOf<PixelRgb> > outPort;
    BufferedPort<Bottle> targetPort;

public:
    void loop()
    {
        ImageOf<PixelRgb> *image=imagePort.read();  // read an image

        if (image!=NULL)
        {
            // check we actually got something
            ImageOf<PixelRgb> &outImage=outPort.prepare(); // get an output image
            outImage=*image;

            double xMean = 0;
            double yMean = 0;
            int ct = 0;
            for (int x=0; x<image->width(); x++)
            {
                for (int y=0; y<image->height(); y++)
                {
                    PixelRgb& pixel = image->pixel(x,y);
                    // very simple test for redishness
                    // make sure red level exceeds blue and green by a factor of 2
                    // plus some threshold
                    if ((pixel.r>pixel.b*5.0) && (pixel.r>pixel.g*5.0))
                    {
                        // there's a redish pixel at (x,y)!
                        // let's find the average location of these pixels

                        // accumulate x
                        // accumulate y
                        // count total number of points
                        xMean += x;
                        yMean += y;
                        ct++;

                        outImage(x,y).r=255;
                    }
                }
            }
            if (ct>0)
            {
                xMean /= ct;
                yMean /= ct;
            }

            Bottle &target=targetPort.prepare();
            target.clear();
            target.addDouble(xMean);
            target.addDouble(yMean);

            //threshold on the size of the object we found
            if (ct>(image->width()/20)*(image->height()/20))
                target.addInt(1);
            else
                target.addInt(0);

            yInfo()<<"Target: "<<target.toString();
            targetPort.write();
            outPort.write();
        }
    }

    bool open()
    {
        bool ret=true;
        ret=imagePort.open("/detector/image/in");  // give the port a name
        ret = ret && outPort.open("/detector/image/out");
        ret = ret && targetPort.open("/detector/target");
        return ret;
    }

    bool close()
    {
        Bottle &target=targetPort.prepare();
        target.clear();
        target.addDouble(0);
        target.addDouble(0);
        target.addInt(0);

        targetPort.writeStrict();
        outPort.write();

        imagePort.close();
        outPort.close();
        targetPort.close();

        return true;
    }

    bool interrupt()
    {
        imagePort.interrupt();
        return true;
    }
};


class DetectorModule: public RFModule
{
   Detector detector;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        return detector.open();
    }

    virtual double getPeriod()
    {
        return 0.0;
    }

    virtual bool updateModule()
    {
        detector.loop();
        return true;
    }

    virtual bool interruptModule()
    {
        yInfo()<<"Interrupting";
        detector.interrupt();
        return true;
    }

    virtual bool close()
    {
        yInfo()<<"Calling close";
        detector.close();
        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    DetectorModule detector;
    return detector.runModule(rf);
}
