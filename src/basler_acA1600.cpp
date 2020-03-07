// Grab.cpp
/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample illustrates how to grab and process images using the CInstantCamera class.
    The images are grabbed and processed asynchronously, i.e.,
    while the application is processing a buffer, the acquisition of the next buffer is done
    in parallel.

    The CInstantCamera class uses a pool of buffers to retrieve image data
    from the camera device. Once a buffer is filled and ready,
    the buffer can be retrieved from the camera object for processing. The buffer
    and additional image data are collected in a grab result. The grab result is
    held by a smart pointer after retrieval. The buffer is automatically reused
    when explicitly released or when the smart pointer object is destroyed.
*/

// Include files to use the PYLON API.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
size_t photo_count = 0;

int main(int argc, char* argv[])
{
    //用于解析ROS参数，第三个参数为本节点名
    ros::init(argc, argv, "basler_acA1600");

    //实例化句柄，初始化node
    ros::NodeHandle nh;

    //消息类
    sensor_msgs::Image photo;
    
    
    //创建publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("basler_acA1600_photo", 1);

    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera.MaxNumBuffer = 5;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing();

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;
        
        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);


            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                cout << "Photo " << photo_count++ << endl;

                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                photo.header = std_msgs::Header();
                photo.height = ptrGrabResult->GetHeight();
                photo.width = ptrGrabResult->GetWidth();
                photo.step = ptrGrabResult->GetImageSize()/photo.height;
                photo.encoding = sensor_msgs::image_encodings::YUV422;
                if(photo.data.size() != ptrGrabResult->GetImageSize())
                {
                    photo.data.resize(ptrGrabResult->GetImageSize());
                }
                for(auto it  = photo.data.begin(); it != photo.data.end(); it++)
                {
                    *it = *(pImageBuffer++);
                }
                
                pub.publish(photo);

#ifdef PYLON_WIN_BUILD
                // Display the grabbed image.
                Pylon::DisplayImage(1, ptrGrabResult);
#endif
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while( cin.get() != '\n');

    // Releases all pylon resources. 
    PylonTerminate();  

    return exitCode;
}
