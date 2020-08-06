//========================================================================================----
//== Motive API DLL
//== Copyright 2010 NaturalPoint, Inc.
//==
//== The Motive API is designed to be a simple yet full featured interface to Motive
//========================================================================================----
#pragma once
#ifndef NPTRACKINGTOOLS_H
#define NPTRACKINGTOOLS_H

//== Includes ============================================================================----
#include <memory>
#include <string>
#include <vector>

#include "RigidBodySolver/RigidBodySettings.h"

//== DLL EXPORT/IMPORT PREPROCESSOR DEFINES ==============================================----

#ifdef NPTRACKINGTOOLS_EXPORTS
    #define TTAPI __declspec(dllexport)
#elif defined NPTRACKINGTOOLS_IMPORTS
    #define TTAPI __declspec(dllimport)
#else
    #define TTAPI
#endif

namespace CameraLibrary
{
    class Camera;
    class CameraManager;
    class cCameraModule;
}

#ifndef _CORE_UID_CLASS
#define _CORE_UID_CLASS

namespace Core
{
    /// <summary>
    /// A platform-neutral 128-bit universal identifier. It is essentially guaranteed to never
    /// generate the same ID twice.
    /// </summary>
    class cUID
    {
    public:
        typedef unsigned long long int uint64;

        /// <summary>
        /// Create a default UID. In order to create a UID that has a valid unique identifier you
        /// must call Generate().
        /// </summary>
        cUID() : mHighBits( 0 ), mLowBits( 0 ) { }

        cUID( uint64 high, uint64 low ) : mHighBits( high ), mLowBits( low ) { }
        cUID( const cUID & obj ) : mHighBits( obj.mHighBits ), mLowBits( obj.mLowBits ) { }
        cUID&           operator=( const cUID & rhs )
        {
            mHighBits = rhs.mHighBits;
            mLowBits = rhs.mLowBits;
            return *this;
        }

        /// <summary>
        /// Set the value of the UID from two long integer values. It is up to the caller to ensure that
        /// the resulting UID is unique.
        /// </summary>
        void            SetValue( uint64 highBits, uint64 lowBits )
        {
            mHighBits = highBits;
            mLowBits = lowBits;
        }

        /// <summary>Get the low 64 bits of the UID.</summary>
        uint64          LowBits() const
        {
            return mLowBits;
        }

        /// <summary>Get the high 64 bits of the UID.</summary>
        uint64          HighBits() const
        {
            return mHighBits;
        }

        /// <summary>Returns true if the ID is valid (i.e. not equal to kInvalid).</summary>
        bool            Valid() const
        {
            return !(mHighBits == 0 && mLowBits == 0);
        }

        /// <summary>Generate a new UID value.</summary>
        static cUID     Generate();

        //==============================================================================================
        // Comparison operators
        //==============================================================================================

        bool            operator<( const cUID & rhs ) const
        {
            return ((mHighBits < rhs.mHighBits) ? true : (mHighBits == rhs.mHighBits ? (mLowBits < rhs.mLowBits) : false));
        }
        bool            operator<=( const cUID & rhs ) const
        {
            return ((mHighBits < rhs.mHighBits) ? true : (mHighBits == rhs.mHighBits ? (mLowBits <= rhs.mLowBits) : false));
        }

        bool            operator>( const cUID & rhs ) const
        {
            return !(*this <= rhs);
        }
        bool            operator>=( const cUID & rhs ) const
        {
            return !(*this < rhs);
        }

        // Inline these for performance.
        bool            operator==( const cUID & rhs ) const
        {
            return ((mHighBits == rhs.mHighBits) && (mLowBits == rhs.mLowBits));
        }

        bool            operator!=( const cUID & rhs ) const
        {
            return ((mHighBits != rhs.mHighBits) || (mLowBits != rhs.mLowBits));
        }

        //==============================================================================================
        // Constants
        //==============================================================================================

        static const cUID kInvalid;

    private:
        uint64          mHighBits;
        uint64          mLowBits;
    };
}
#endif // _CORE_UID_CLASS

#ifndef _CORE_LABEL_CLASS
#define _CORE_LABEL_CLASS
namespace Core
{
    /// <summary>A class that represents a marker label. Marker labels consist of two parts: The entity that the marker
    /// is associated with (e.g. skeleton, rigid body, etc.), and the (one-based) index into the label list for that entity.
    class cLabel
    {
    public:
        cLabel();
        cLabel( const cUID& entityID, unsigned int memberLabelIndex );
        cLabel( const cLabel& other );

        cLabel&         operator=( const cLabel& other );

        /// <summary>The node ID for the entity that this label belongs to.</summary>
        const cUID&     EntityID() const { return mEntityID; }

        /// <summary>The label ID within the entity.</summary>
        unsigned int    MemberID() const { return mMemberLabelID; }

        /// <summary>True if the label has a non-null entity ID. Does not attempt to ensure that the entity ID
        /// is valid or that it belongs to an asset that has associated markers for labeling.</summary>
        bool            Valid() const { return ( mEntityID != cUID::kInvalid ); }

        /// <summary>Parse fully qualified name into entity and member names.</summary>
        static void     ParseName( const std::wstring &name, std::wstring &entityName, std::wstring &memberName );

        /// <summary>Comparison operators.</summary>
        bool            operator==( const cLabel& other ) const;
        bool            operator!=( const cLabel& other ) const;
        bool            operator<( const cLabel & rhs ) const
        {
            return ( ( mEntityID < rhs.mEntityID ) ? true : ( mEntityID == rhs.mEntityID ? ( mMemberLabelID < rhs.mMemberLabelID ) : false ) );
        }

        // Convenience constants
        static const cLabel kInvalid;

    private:
        cUID            mEntityID;
        unsigned int    mMemberLabelID;

        // Legacy items for handling the old UID-based labels, mostly for deserializing older takes.
        static const long long klabelIdentifier;
        static const long long kTypeMask;
        static bool     LegacyIsLabel( const Core::cUID& uid, bool checkForValidType = false );
    };
}
#endif // _CORE_LABEL_CLASS

#ifndef _CORE_MARKER_CLASS
#define _CORE_MARKER_CLASS
namespace Core
{
    template <typename T>
    class cTMarker
    {
    public:
        cTMarker() : X( 0 ), Y( 0 ), Z( 0 ), ID( cUID::kInvalid ), Size( 0 ), Label( cLabel::kInvalid ), Residual( 0 ), Selected( false ),
            Synthetic( false ) { }
        cTMarker( T x, T y, T z ) : X( x ), Y( y ), Z( z ), Size( 0 ), Label( cLabel::kInvalid ), Residual( 0 ), Selected( false ),
            ID( cUID::kInvalid ), Synthetic( false ) { }
        cTMarker( const cTMarker &base ) : X( base.X ), Y( base.Y ), Z( base.Z ), ID( base.ID ), Size( base.Size ), Label( base.Label ),
            Selected( base.Selected ), Residual( base.Residual ), Synthetic( base.Synthetic )
        {
        }

        cTMarker&        operator=( const cTMarker &base )
        {
            X = base.X;
            Y = base.Y;
            Z = base.Z;

            ID = base.ID;
            Size = base.Size;
            Label = base.Label;
            Selected = base.Selected;
            Residual = base.Residual;
            Synthetic = base.Synthetic;

            return *this;
        }

        bool            operator==( const cTMarker& other ) { return ( Label == other.Label ); }
        bool            operator!=( const cTMarker& other ) { return ( Label != other.Label ); }

        /// <summary>Set the position.</summary>
        void            SetPosition( T x, T y, T z ) { X = x; Y = y; Z = z; }

        /// <summary>Returns true if this was recorded from an active marker.</summary>
        bool            IsActiveMarker() const { return ( ID.LowBits() == kActiveMarkerIDTag ); }

        T               X;              //== Position in meters
        T               Y;              //== Position in meters
        T               Z;              //== Position in meters
        cUID            ID;             //== Marker ID (which may be assigned during reconstruction)
        T               Size;           //== Diameter in meters
        cLabel          Label;          //== Marker Label
        bool            Selected;       //== Selection state
        T               Residual;       //== Residual in mm/ray
        bool            Synthetic;      //== Synthetic markers created in pipeline such as virtual finger tip markers

        // When a marker is actively labeled, it will have an ID that includes this constant.
        static const unsigned int kActiveMarkerIDTag = 0x8d403b2a; // Do not change this value.
    };

    typedef cTMarker<float> cMarker;
    typedef cTMarker<float> cMarkerf;
    typedef cTMarker<double> cMarkerd;
}
#endif // _CORE_MARKER_CLASS

//== PUBLIC INTERFACE ====================================================================----

#define NPRESULT int                  //== NPRESULT Defines Call Success/Failure =========----

//== STARTUP / SHUTDOWN ==================================================================----

TTAPI   NPRESULT TT_Initialize();      //== Initialize Library ===========================----
TTAPI   NPRESULT TT_Shutdown();        //== Shutdown Library =============================----

TTAPI   NPRESULT TT_TestSoftwareMutex(); //== Determine if no other OptiTrack software ===----
                                         //== is currently using devices.  ===============----

//== FRAME PROCESSING ====================================================================----

TTAPI   NPRESULT TT_Update();                          //== Process incoming camera data =----
TTAPI   NPRESULT TT_UpdateSingleFrame();               //== Process incoming camera data =----
TTAPI   NPRESULT TT_UpdateLastestFrame();              //== Process only the most recent =----
                                                       //== camera data frame and        =----
                                                       //== discard rest.                =----

//== PROFILE INTERFACE ===================================================================----

TTAPI   NPRESULT TT_LoadProfile( const char *filename );     //== Load User Profile File =----
TTAPI   NPRESULT TT_LoadProfileW( const wchar_t *filename ); 
TTAPI   NPRESULT TT_SaveProfile( const char *filename );     //== Save User Profile File =----
TTAPI   NPRESULT TT_SaveProfileW( const wchar_t *filename ); 

//== CAMERA CALIBRATION INTERFACE ========================================================----

TTAPI   NPRESULT TT_LoadCalibrationW( const wchar_t *filename ); //== Load Calibration ===----
TTAPI   NPRESULT TT_LoadCalibration( const char *filename );
TTAPI   NPRESULT TT_LoadCalibrationFromMemory( unsigned char* buffer, int bufferSize );

//== LICENSING ===========================================================================----

                 //== Licenses are automatically loaded from the OptiTrack license directory.
                 //== This is not needed except to accommodate some very rare user
                 //==  scenarios.  Call this and provide the contents of a license
                 //== file located outside the license folder.  Call this function before
                 //== TT_Initialize();

TTAPI   NPRESULT TT_LoadLicenseFromMemory( const unsigned char * buffer, int bufferSize );

//== RIGID BODY INTERFACE ================================================================----

TTAPI   NPRESULT TT_LoadRigidBodiesW( const wchar_t *filename ); //== Load Rigid Bodies ==----
TTAPI   NPRESULT TT_LoadRigidBodies( const char *filename );
TTAPI   NPRESULT TT_SaveRigidBodiesW( const wchar_t *filename ); //== Save Rigid Bodies ==----
TTAPI   NPRESULT TT_SaveRigidBodies( const char *filename );
TTAPI   NPRESULT TT_AddRigidBodiesW( const wchar_t *filename ); //== Add Rigid Bodies ====----
TTAPI   NPRESULT TT_AddRigidBodies( const char *filename );

//== DATA STREAMING ======================================================================----

TTAPI   NPRESULT TT_StreamTrackd( bool enabled );         //== Start/stop Trackd Stream ==----
TTAPI   NPRESULT TT_StreamVRPN( bool enabled, int port ); //== Start/stop VRPN Stream ====----
TTAPI   NPRESULT TT_StreamNP( bool enabled );             //== Start/stop NaturalPoint Stream 

//== FRAME ===============================================================================----

TTAPI   int        TT_FrameMarkerCount();               //== Returns Frame Markers Count =----
TTAPI   float      TT_FrameMarkerX( int markerIndex );  //== Returns X Coord of Marker ===----
TTAPI   float      TT_FrameMarkerY( int markerIndex );  //== Returns Y Coord of Marker ===----
TTAPI   float      TT_FrameMarkerZ( int markerIndex );  //== Returns Z Coord of Marker ===----
TTAPI   Core::cUID TT_FrameMarkerLabel( int markerIndex ); //== Returns Label of Marker ==----
TTAPI   float      TT_FrameMarkerResidual( int markerIndex ); //== Returns Residual of Marker
TTAPI   double     TT_FrameTimeStamp();                 //== Time Stamp of Frame (seconds)
TTAPI   int        TT_FrameID();                        //== FrameID of Frame

              //== TT_FrameCameraCentroid returns true if the camera is contributing
              //== to this 3D marker.  It also returns the location of the 2D centroid
              //== that is reconstructing to this 3D marker.

TTAPI   bool  TT_FrameCameraCentroid( int markerIndex, int cameraIndex, float &x, float &y );

              //== In the event that you are tracking a very high number of 2D and/or
              //== 3D markers (hundreds of 3D markers), and you find that the data
              //== you're getting out has sufficient latency you can call
              //== TT_FlushCameraQueues() to catch up before calling TT_Update().
              //== Ideally, after calling TT_FlushCameraQueues() you'll want to
              //== not call it again until after TT_Update() returns NPRESULT_SUCCESS

TTAPI   void  TT_FlushCameraQueues();

//== RIGID BODY CONTROL ==================================================================----

TTAPI   bool     TT_IsRigidBodyTracked( int rbIndex );      //== Is rigid body currently tracked
TTAPI   void     TT_RigidBodyLocation( int rbIndex,         //== RigidBody Index =========----
    float *x, float *y, float *z,                           //== Position    =============----
    float *qx, float *qy, float *qz, float *qw,             //== Orientation =============----
    float *yaw, float *pitch, float *roll );                //== Orientation =============----

TTAPI   void     TT_ClearRigidBodyList();                   //== Clear all rigid bodies ==----
TTAPI   NPRESULT TT_RemoveRigidBody( int rbIndex );         //== Remove single rigid body ----
TTAPI   int      TT_RigidBodyCount();                       //== Returns number of rigid bodies

TTAPI   int      TT_RigidBodyUserData( int rbIndex );       //== Get RigidBodies User Data ---
TTAPI   void     TT_SetRigidBodyUserData( int rbIndex, int ID ); //== Set RigidBodies User Data
TTAPI   const char* TT_RigidBodyName( int rbIndex );        //== Returns RigidBody Name ==----
TTAPI   const wchar_t* TT_RigidBodyNameW( int rbIndex );    //== Returns RigidBody Name ==----

TTAPI   void     TT_SetRigidBodyEnabled( int rbIndex, bool enabled ); //== Set Tracking ==----
TTAPI   bool     TT_RigidBodyEnabled( int rbIndex );        //== Get Tracking ============----

TTAPI   NPRESULT TT_RigidBodyTranslatePivot( int rbIndex, float x, float y, float z );
TTAPI   bool     TT_RigidBodyResetOrientation( int rbIndex );

TTAPI   int      TT_RigidBodyMarkerCount( int rbIndex );    //== Get marker count ========----
TTAPI   bool     TT_RigidBodyMarker( int rbIndex, int markerIndex, //== Get RigidBody mrkr ---
    float *x, float *y, float *z );
TTAPI   bool     TT_RigidBodyUpdateMarker( int rbIndex, int markerIndex, //== Update RigidBody mrkr
    float *x, float *y, float *z );

TTAPI   bool     TT_RigidBodyPointCloudMarker( int rbIndex, //== Get corresponding point cloud marker
    int markerIndex, bool &tracked,                         //== If tracked is false, there is no
    float &x, float &y, float &z );                         //== corresponding point cloud marker.

TTAPI   bool     TT_RigidBodyPlacedMarker( int rbIndex,     //== Get world location of the rigid body
    int markerIndex, bool &tracked,                         //== marker if tracked.
    float &x, float &y, float &z );                         

TTAPI   float    TT_RigidBodyMeanError( int rbIndex );      //== Get mean error of tracked rigid
                                                            //== body. (in meters)

TTAPI   Core::cUID  TT_RigidBodyID( int rbIndex );          //== Get rigid body cUID.

//== Create a rigid body based on the marker count and marker list provided.  The marker list is
//== expected to contain of list of marker coordinates in the order: x1,y1,z1,x2,y2,z2,...xN,yN,zN.
TTAPI   NPRESULT TT_CreateRigidBody( const char* name, int id, int markerCount, float *markerList );

TTAPI   NPRESULT TT_RigidBodySettings( int rbIndex, RigidBodySolver::cRigidBodySettings &settings );  //== Get RigidBody Settings =---
TTAPI   NPRESULT TT_SetRigidBodySettings( int rbIndex, RigidBodySolver::cRigidBodySettings &settings );  //== Set RigidBody Settings =---

//== CAMERA MANAGER ACCESS ===============================================================----

TTAPI   CameraLibrary::CameraManager* TT_GetCameraManager(); //== Returns a pointer to the Camera SDK's
                                                            //== CameraManager ===========----

TTAPI   int	     TT_BuildNumber();                          //== Software Release Build # ----

//== CAMERA GROUP SUPPORT ================================================================----

TTAPI   int      TT_CameraGroupCount();                     //== Returns number of camera groups
TTAPI   bool     TT_CreateCameraGroup();                    //== Add an additional group =----
TTAPI   bool     TT_RemoveCameraGroup( int groupIndex );    //== Remove a camera group (must be empty)
TTAPI   int      TT_CamerasGroup( int cameraIndex );        //== Returns Camera's camera group index

TTAPI   void     TT_SetGroupShutterDelay( int groupIndex, int microseconds ); //== Set camera group's shutter delay
TTAPI   void     TT_SetCameraGroup( int cameraIndex, int groupIndex ); //== Move camera to camera group

//== CAMERA GROUP FILTER SETTINGS ========================================================----

class TTAPI cCameraGroupFilterSettings
{
public:
    cCameraGroupFilterSettings();
    ~cCameraGroupFilterSettings();

    enum eFilterType
    {
        FilterNone,
        FilterSizeRoundness,
        FilterCount
    };

    eFilterType  FilterType;
    int          MinMarkerSize;
    int          MaxMarkerSize;
    float        MinRoundness;
};

TTAPI   NPRESULT TT_CameraGroupFilterSettings( int groupIndex, cCameraGroupFilterSettings &settings );
TTAPI   NPRESULT TT_SetCameraGroupFilterSettings( int groupIndex, cCameraGroupFilterSettings &settings );

//== Point Cloud Reconstruction Settings ==---
class TTAPI cCameraGroupPointCloudSettings
{
public:
    enum Setting : unsigned long long
    {
        eResolvePointCloud = 1LL,               // bool
        eShowCameras = 1LL << 1,                // bool
        eVisibleMarkerSize = 1LL << 2,          // double
        ePCResidual = 1LL << 3,                 // double
        ePCMinAngle = 1LL << 4,                 // double
        ePCMinRays = 1LL << 5,                  // long
        eShutterDelay = 1LL << 6,               // long
        ePrecisionPacketCap = 1LL << 7,         // long
        ePCMinRayLength = 1LL << 8,             // double
        ePCMaxRayLength = 1LL << 9,             // double
        ePCReconstructXCenter = 1LL << 10,      // double
        ePCReconstructYCenter = 1LL << 11,      // double
        ePCReconstructZCenter = 1LL << 12,      // double
        ePCReconstructXWidth = 1LL << 13,       // double
        ePCReconstructYWidth = 1LL << 14,       // double
        ePCReconstructZWidth = 1LL << 15,       // double
        ePCReconstructRadius = 1LL << 16,       // double
        ePCReconstructHeight = 1LL << 17,       // double
        ePCReconstructShape = 1LL << 18,        // shape 0=Cuboid,1=Spherical,2=Cylindrical,3=Ellipsoid
        ePCObjectFilterLevel = 1LL << 19,       // long
        ePCObjectFilterMinSize = 1LL << 20,     // long
        ePCObjectFilterMaxSize = 1LL << 21,     // long
        ePCObjectFilterCircularity = 1LL << 22, // double
        ePCObjectFilterGrayscaleFloor = 1LL << 23, // long
        ePCObjectFilterAspectTolerance = 1LL << 24, // long
        ePCObjectFilterObjectMargin = 1LL << 25, // long
        eShowReconstructionBounds = 1LL << 26,  // bool
        eBoundReconstruction = 1LL << 27,       // bool
        eShowCaptureVolume = 1LL << 28,         // bool
        eShow3DMarkers = 1LL << 29,             // bool
        eShowCameraFOV = 1LL << 30,             // bool
        eCameraOverlap = 1LL << 31,             // double
        eVolumeResolution = 1LL << 32,          // double
        eWireframe = 1LL << 33,                 // double
        eFOVIntensity = 1LL << 34,              // double
        ePCPixelGutter = 1LL << 35,             // long
        ePCMaximum2DPoints = 1LL << 36,         // long
        eBlockWidth = 1LL << 37,                // double
        eBlockHeight = 1LL << 38,               // double
        eSynchronizerEngine = 1LL << 39,        // long 1=v1.0  2=v2.0
        eMarkerDiameterType = 1LL << 40,        // long
        eMarkerDiameterForceSize = 1LL << 41,   // double
        eSynchronizerControl = 1LL << 42,       // long
        eContinuousCalibration = 1LL << 43,     // long
        eSettingsCount
    };

    cCameraGroupPointCloudSettings();
    ~cCameraGroupPointCloudSettings();

    //== Set individual parameter values. Only values that are set will be changed when submitting
    //== the settings block to TT_SetCameraGroupPointCloudSettings. These methods will return false 
    //== if there is a mismatch between the requested parameter and its expected type.
    bool            SetBoolParameter( Setting which, bool val );
    bool            SetDoubleParameter( Setting which, double val );
    bool            SetLongParameter( Setting which, long val );

    //== Retrieve individual parameter settings from the parameter block. These methods will return false 
    //== if there is a mismatch between the requested parameter and its expected type.
    bool            BoolParameter( Setting which, bool &val ) const;
    bool            DoubleParameter( Setting which, double &val ) const;
    bool            LongParameter( Setting which, long &val ) const;

private:
    unsigned long long mWhichSet;
    void *mSettings;

    friend TTAPI NPRESULT TT_CameraGroupPointCloudSettings( int groupIndex, cCameraGroupPointCloudSettings &settings );
    friend TTAPI NPRESULT TT_SetCameraGroupPointCloudSettings( int groupIndex, cCameraGroupPointCloudSettings &settings );
};

TTAPI   NPRESULT TT_CameraGroupPointCloudSettings( int groupIndex, cCameraGroupPointCloudSettings &settings );

// THIS METHOD IS Deprecated. The sync group and cCameraGroupPointCloudSettings now reference the same settings which
// obviates the the need to ever invoke this function. It is left in as a no-op for backward compatibility.
TTAPI   NPRESULT TT_SetCameraGroupPointCloudSettings( int groupIndex, cCameraGroupPointCloudSettings &settings );

//== Marker Size Settings ====----

class TTAPI cCameraGroupMarkerSizeSettings
{
public:
    cCameraGroupMarkerSizeSettings();
    ~cCameraGroupMarkerSizeSettings();

    enum eMarkerSizeType
    {
        MarkerSizeCalculated,
        MarkerSizeFixed,
        MarkerSizeCount
    };

    eMarkerSizeType  MarkerSizeType;
    float            MarkerSize;
};

TTAPI   NPRESULT TT_CameraGroupMarkerSize( int groupIndex, cCameraGroupMarkerSizeSettings &settings );
TTAPI   NPRESULT TT_SetCameraGroupMarkerSize( int groupIndex, cCameraGroupMarkerSizeSettings &settings );

TTAPI   NPRESULT TT_SetCameraGroupReconstruction( int groupIndex, bool enable );

TTAPI   NPRESULT TT_SetEnabledFilterSwitch( bool enabled ); //== Enabled by default ========--
TTAPI   bool     TT_IsFilterSwitchEnabled();

//== POINT CLOUD INTERFACE ==============================================================-----

TTAPI   int      TT_CameraCount();                          //== Returns Camera Count ===-----
TTAPI   float    TT_CameraXLocation( int cameraIndex );     //== Returns Camera's X Coord ----
TTAPI   float    TT_CameraYLocation( int cameraIndex );     //== Returns Camera's Y Coord ----
TTAPI   float    TT_CameraZLocation( int cameraIndex );     //== Returns Camera's Z Coord ----
TTAPI   float    TT_CameraOrientationMatrix( int cameraIndex, int matrixIndex ); //== Orientation

TTAPI   const char* TT_CameraName( int cameraIndex );       //== Returns Camera Name =====----
TTAPI   int      TT_CameraSerial( int cameraIndex );        //== Returns Camera Serial Number

TTAPI   int      TT_CameraMarkerCount( int cameraIndex );   //== Camera's 2D Marker Count ----

//== CameraMarker fetches the 2D centroid location of the marker as seen by the camera.
TTAPI   bool     TT_CameraMarker( int cameraIndex, int markerIndex, float &x, float &y );

//== Camera Pixel Resolution ==--

TTAPI   bool     TT_CameraPixelResolution( int cameraIndex, int &width, int &height );

//== Fetch pre-distorted marker location.  This is basically where the camera would see the marker if
//== there was no lens distortion. For most of our cameras/lenses, this location is only a few pixels
//== from the distorted (TT_CameraMarker) position.
TTAPI   bool     TT_CameraMarkerPredistorted( int cameraIndex, int markerIndex, float &x, float &y );

//== Set camera settings.  This function allows you to set the camera's video mode, exposure, threshold,
//== and illumination settings.
                 
//== VideoType:  
//==     0 = Segment Mode   
//==     1 = Grayscale Mode 
//==     2 = Object Mode    
//==     4 = Precision Mode
//==     6 = MJPEG Mode

//== Exposure: Valid values are:  1-480
//== Threshold: Valid values are: 0-255
//== Intensity: Valid values are: 0-15  (This should be set to 15 for most situations)
TTAPI   bool     TT_SetCameraSettings( int cameraIndex, int videoType, int exposure, int threshold, int intensity );

//== Set the frame rate for the given zero based camera index. Returns true if the operation was successful
//== and false otherwise. If the operation fails check that the camera index is valid and that devices have
//== been initialized with TT_Initialize().
TTAPI   bool     TT_SetCameraFrameRate( int cameraIndex, int frameRate );

//== Get camera settings for a given camera index. A negative return value indicates the value was not
//== available. This usually means that either the camera index is not valid or devices have not been
//== initialized with TT_Initialize()

//==     0 = Segment Mode   
//==     1 = Grayscale Mode 
//==     2 = Object Mode    
//==     4 = Precision Mode
//==     6 = MJPEG Mode
TTAPI   int      TT_CameraVideoType( int cameraIndex );

TTAPI   int      TT_CameraFrameRate( int cameraIndex ); // frames/sec
TTAPI   int      TT_CameraExposure( int cameraIndex );
TTAPI   int      TT_CameraThreshold( int cameraIndex );
TTAPI   int      TT_CameraIntensity( int cameraIndex );
TTAPI   float    TT_CameraTemperature( int cameraIndex );
TTAPI   float    TT_CameraRinglightTemperature( int cameraIndex );

//== Camera's Full Frame Grayscale Decimation =---
TTAPI   int      TT_CameraGrayscaleDecimation( int cameraIndex );
TTAPI   bool     TT_SetCameraGrayscaleDecimation( int cameraIndex, int value );

//== Toggle camera extended options
TTAPI   bool     TT_SetCameraFilterSwitch( int cameraIndex, bool enableIRFilter );
TTAPI   bool     TT_SetCameraAGC( int cameraIndex, bool enableAutomaticGainControl );
TTAPI   bool     TT_SetCameraAEC( int cameraIndex, bool enableAutomaticExposureControl );
TTAPI   bool     TT_SetCameraHighPower( int cameraIndex, bool enableHighPowerMode );
TTAPI   bool     TT_SetCameraMJPEGHighQuality( int cameraIndex, int mjpegQuality );

//== Camera Imager Gain ==--
TTAPI   int      TT_CameraImagerGain( int cameraIndex );
TTAPI   int      TT_CameraImagerGainLevels( int cameraIndex );
TTAPI   void     TT_SetCameraImagerGain( int cameraIndex, int value );

//== Camera Illumination ==--
TTAPI   bool     TT_IsContinuousIRAvailable( int cameraIndex );
TTAPI   bool     TT_ContinuousIR( int cameraIndex );
TTAPI   void     TT_SetContinuousIR( int cameraIndex, bool Enable );

//== Camera Masking ==--

TTAPI  bool      TT_ClearCameraMask( int cameraIndex );
TTAPI  bool      TT_SetCameraMask( int cameraIndex, unsigned char * buffer, int bufferSize );
TTAPI  bool      TT_CameraMask( int cameraIndex, unsigned char * buffer, int bufferSize );
TTAPI  bool      TT_CameraMaskInfo( int cameraIndex, int &blockingMaskWidth, int &blockingMaskHeight, int &blockingMaskGrid );

//== Camera State ==--

enum eCameraStates
{
    Camera_Enabled = 0,
    Camera_Disabled_For_Reconstruction = 1,
    Camera_Disabled = 2,
    CameraStatesCount = 3
};

TTAPI  bool      TT_SetCameraState( int cameraIndex, eCameraStates state );
TTAPI  bool      TT_CameraState( int cameraIndex, eCameraStates &currentState );

//== Camera ID ==--
TTAPI   int      TT_CameraID( int cameraIndex );

//== Fetch the camera's frame buffer.  This function fills the provided buffer with an image of what
//== is in the camera view.  The resulting image depends on what video mode the camera is in.  If
//== the camera is in grayscale mode, for example, a grayscale image is returned from this call.
TTAPI   bool     TT_CameraFrameBuffer( int cameraIndex, int bufferPixelWidth, int bufferPixelHeight,
    int bufferByteSpan, int bufferPixelBitDepth, unsigned char *buffer );

//== Save camera's frame buffer as a BMP image file
TTAPI   bool     TT_CameraFrameBufferSaveAsBMP( int cameraIndex, const char *filename );

//== Back-project from 3D space to 2D space.  If you give this function a 3D location and select a
//== camera, it will return where the point would land on the imager of that camera in to 2D space.
//== This basically locates where in the camera's FOV a 3D point would be located.
TTAPI   void     TT_CameraBackproject( int cameraIndex, float x, float y, float z, float &cameraX, float &cameraY );

//== The 2D centroids the camera reports are distorted by the lens.  To remove the distortion call
//== CameraUndistort2DPoint.  Also if you have a 2D undistorted point that you'd like to convert back
//== to a distorted point call CameraDistort2DPoint.
TTAPI   void     TT_CameraUndistort2DPoint( int cameraIndex, float &x, float &y );
TTAPI   void     TT_CameraDistort2DPoint( int cameraIndex, float &x, float &y );

//== Takes an undistorted 2D centroid and return a camera ray in the world coordinate system.
TTAPI   bool     TT_CameraRay( int cameraIndex, float x, float y, float &rayStartX, float &rayStartY, float &rayStartZ,
    float &rayEndX, float &rayEndY, float &rayEndZ );

//== Set a camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters with
//== parameters compatible with the OpenCV intrinsic model.
TTAPI   bool     TT_CameraModel( int cameraIndex, float x, float y, float z, //== Camera Position
    float *orientation,                                     //== Orientation (3x3 matrix) ----
    float principleX, float principleY,                     //== Lens center (in pixels) =----
    float focalLengthX, float focalLengthY,                 //== Lens focal  (in pixels) =----
    float kc1, float kc2, float kc3,                        //== Barrel distortion coefficients
    float tangential0, float tangential1 );                 //== Tangential distortion ===----

//== Set a camera's extrinsic (position & orientation).
TTAPI   bool     TT_CameraPose( int cameraIndex, float x, float y, float z,  //== Camera Position
    float *orientation );                                   //== Orientation (3x3 matrix) ----

//== Get camera extrinsics from a calibration file in memory. This allows for acquiring camera
//== extrinsics for cameras not connected to system.  It simply returns the list of details for all
//== cameras contained in the calibration file.

struct TT_sCameraInfo
{
    int CameraSerial;
    float Position[ 3 ];
    float Orientation[ 9 ];
};

class TTAPI TT_cCameraList
{
public:
    TT_cCameraList();

    //== CameraCount() returns the number of cameras present in TT_cCameraList

    int CameraCount() const;

    //== Camera() returns a TT_sCameraInfo structure that contains camera
    //== serial number, position, and orientation information.

    TT_sCameraInfo Camera( int index ) const;

    void AddCamera( const TT_sCameraInfo & cameraInfo );

private:
    std::vector<TT_sCameraInfo> mCameraList;
};

TTAPI   TT_cCameraList TT_CameraExtrinsicsCalibrationFromMemory( unsigned char* Buffer, int BufferSize
                                                    , NPRESULT & result  );


//== Additional Functionality ===========================================================-----

//== This function will return the Camera SDK's camera pointer.  While the API takes over the
//== data path which prohibits fetching the frames directly from the camera, it is still very usefuL to be
//== able to communicate with the camera directly for setting camera settings or attaching modules.
TTAPI   CameraLibrary::Camera * TT_GetCamera( int cameraIndex );  //== Returns Camera SDK Camera

TTAPI   bool TT_SetFrameIDBasedTiming( bool enable );
TTAPI   bool TT_SetSuppressOutOfOrder( bool enable );

TTAPI   void TT_AttachCameraModule( int cameraIndex, CameraLibrary::cCameraModule *module );
TTAPI   void TT_DetachCameraModule( int cameraIndex, CameraLibrary::cCameraModule *module );

TTAPI   NPRESULT TT_OrientTrackingBar( float positionX, float positionY, float positionZ,
    float orientationX, float orientationY, float orientationZ, float orientationW );

//== Rigid Body Solver Callback Hook ====================================================-----

//== Inherit cRigidBodySolutionTest and overload the RigidBodySolutionTest method to have the ability to reject
//== potential rigid body solutions during the rigid body solving process.  You must attach your cRigidBodySolutionTest
//== class to a rigid body via TT_AttachRigidBodySolutionTest.  Return false if the presented solution should be
//== rejected.

class TTAPI cRigidBodySolutionTest
{
public:
    cRigidBodySolutionTest()  {}
    virtual ~cRigidBodySolutionTest() {}

    //== RigidBody Solution Test ==--

    virtual bool RigidBodySolutionTest( int markerCount, Core::cMarker *markers, bool *markerExists ) { return true; }
};

TTAPI void TT_AttachRigidBodySolutionTest( int rbIndex, cRigidBodySolutionTest* test );
TTAPI void TT_DetachRigidBodySolutionTest( int rbIndex, cRigidBodySolutionTest* test );

//== Rigid Body Refinement ==============================================================-----

//== You will want to get a rigid body's ID from TT_RigidBodyID for use with the following functions.  To start the refine process,
//== call TT_RigidBodyRefineStart with the rigid body's ID along with the number of samples you'd like to take before the refinement is
//== performed.

TTAPI bool TT_RigidBodyRefineStart( Core::cUID rigidBodyID, int sampleCount );

//== Call TT_RigidBodyRefineSample() every frame after calling TT_RigiDBodyRefineStart. This will allow the refinement process to collect
//== samples.  You can check the progress of samples by calling TT_RigidBodyRefineProgress() and it will report a percentage of the total
//== samples collected.  The refinement process will not collect samples when the rigid body is untracked.

TTAPI bool TT_RigidBodyRefineSample();

//== You can query the state of the refinement process with TT_RigidBodyRefineState().

enum TT_RigidBodyRefineStates
{
    TT_RigidBodyRefine_Initialized = 0,
    TT_RigidBodyRefine_Sampling,
    TT_RigidBodyRefine_Solving,
    TT_RigidBodyRefine_Complete,
    TT_RigidBodyRefine_Uninitialized
};

TTAPI TT_RigidBodyRefineStates TT_RigidBodyRefineState();

//== To ensure the refinement solver is collecting samples, call TT_RigidBodyRefineProgress() during TT_RigidBodyRefine_Sampling state.  Progress
//== is reported as a percentage of the total samples during sampling.

TTAPI float TT_RigidBodyRefineProgress();

//== Once TT_RigidBodyRefine_Complete state is reached, you can use TT_RigidBodyRefineInitialError() and TT_RigidBodyRefineResultError() to determine
//== if the result has improved prior to calling TT_RigidBodyRefineApplyResult().

TTAPI float TT_RigidBodyRefineInitialError();
TTAPI float TT_RigidBodyRefineResultError();

//== Apply the resulting rigid body refinement result by calling TT_RigidBodyRefineApplyResult().

TTAPI bool  TT_RigidBodyRefineApplyResult();

//== To discard the rigid body refinement result, call TT_RigidBodyRefineReset().

TTAPI bool  TT_RigidBodyRefineReset();

//== Rigid Body Pivot Location Solver =========================================================-----

//== You will want to get a rigid body's ID from TT_RigidBodyID for use with the following functions.  To start the pivot solving process,
//== call TT_RigidBodyPivotSolverStart with the rigid body's ID along with the number of samples you'd like to take before the solving is
//== performed.

TTAPI bool TT_RigidBodyPivotSolverStart( Core::cUID rigidBodyID, int sampleCount );

//== Call TT_RigidBodyPivotSolverSample() every frame after calling TT_RigidBodyPivotSolverStart. This will allow the solving process to collect
//== samples.  You can check the progress of samples by calling TT_RigidBodyPivotSolverProgress() and it will report a percentage of the total
//== samples collected.  The solving process will not collect samples when the rigid body is untracked.

TTAPI bool TT_RigidBodyPivotSolverSample();

//== You can query the state of the refinement process with TT_RigidBodyPivotSolverState().

enum TT_RigidBodyPivotSolverStates
{
    TT_RigidBodyPivotSolver_Initialized = 0,
    TT_RigidBodyPivotSolver_Sampling,
    TT_RigidBodyPivotSolver_Solving,
    TT_RigidBodyPivotSolver_Complete,
    TT_RigidBodyPivotSolver_Uninitialized
};

TTAPI TT_RigidBodyPivotSolverStates TT_RigidBodyPivotSolverState();

//== To ensure the refinement solver is collecting samples, call TT_RigidBodyPivotSolverProgress() during TT_RigidBodyPivotSolver_Sampling state.  Progress
//== is reported as a percentage of the total samples during sampling.

TTAPI float TT_RigidBodyPivotSolverProgress();

//== Once TT_RigidBodyPivotSolver_Complete state is reached, you can use TT_RigidBodyPivotSolverInitialError() and TT_RigidBodyPivotSolverResultError() to determine
//== if the result has improved prior to calling TT_RigidBodyPivotSolverApplyResult().

TTAPI float TT_RigidBodyPivotSolverInitialError();
TTAPI float TT_RigidBodyPivotSolverResultError();

//== Apply the resulting rigid body refinement result by calling TT_RigidBodyPivotSolverApplyResult().

TTAPI bool  TT_RigidBodyPivotSolverApplyResult();

//== To discard the rigid body refinement result, call TT_RigidBodyPivotSolverReset().

TTAPI bool  TT_RigidBodyPivotSolverReset();

//== Functionality to set pixel intensity weighting for 2D centroid calculation LUT.

TTAPI bool  TT_SetPixelIntensityMapping( int grayscaleFloor, int mapEntryCount, float *map );

//== API Callbacks ======================================================================-----

//== Inherit cTTAPIListener and override it's methods to receive callbacks from the TTAPI.  You must attach your
//== listening class to the TTAPI via TT_AttachListener.

class TTAPI cTTAPIListener
{
public:
    cTTAPIListener();
    virtual ~cTTAPIListener();

    //== TTAPIFrameAvailable callback is called when a new synchronized group of camera frames has been delivered
    //== to the TTAPI and is ready for processing.  You can use this notification to then call TT_Update() without
    //== having to poll blindly for new data.

    virtual void TTAPIFrameAvailable();

    //== TTAPICameraConnected callback is called when a camera is connected.

    virtual void TTAPICameraConnected( int serialNumber );

    //== TTAPICameraConnected callback is called when a camera is connected.

    virtual void TTAPICameraDisconnected( int serialNumber );

    //== InitialPointCloud is called when the initial point cloud is calculated from the connected cameras. During
    //== this callback 3D markers can be added (up to MaxMarkers) or removed by modifying the Markers list as well
    //== as the MarkerCount variable.  After this callback the marker list is passed onto the rigid body solver.

    virtual void InitialPointCloud( Core::cMarker* markers, int &markerCount, int maxMarkers );

    //== When Continuous Calibration is enabled, this callback when an updated calibration is ready to be applied
    //== to the cameras.  This callback can be used to prevent the application of the updated calibration. Return
    //== true to apply the updated calibration. Return false to toss the updated calibration.  Continuous Calibration
    //== if enabled will continue and will call this callback when the next updated calibration is ready for application.

    virtual bool ApplyContinuousCalibrationResult();

    //== Rigid body solving callbacks to adjust marker weighting.  When the rigid body is being solved in marker
    //== based solving, the marker weights can be adjusted.  During this callback, markerUtilization is an array of
    //== booleans describing which markers are tracked and used on this frame.  Both the rigid body's markerLocations
    //== and the associated point cloud locations are also passed.  The markerWeights array can be modified in this
    //== callback and the final rigid body placement will be based on these updated markerWeights.  If the weights
    //== are modified return true from this function to signify that the weighting has been changed.

    virtual bool RigidBodyMarkerWeights( Core::cUID rigidBodyID, int markerCount, bool * markerUtilization, float * markerWeights
                                       , Core::cMarkerf * markerLocations, Core::cMarkerf * pointCloudLocations );
};

TTAPI   void     TT_AttachListener( cTTAPIListener* listener );
TTAPI   void     TT_DetachListener( cTTAPIListener* listener );

//== RESULT PROCESSING ===================================================================----

TTAPI   const char* TT_GetResultString( NPRESULT result ); //== Return Plain Text Message ----
#define NPRESULT_SUCCESS                0             //== Successful Result =============----
#define NPRESULT_FILENOTFOUND           1             //== File Not Found ================----
#define NPRESULT_LOADFAILED             2             //== Load Failed ===================----
#define NPRESULT_FAILED                 3             //== Failed ========================----
#define NPRESULT_INVALIDFILE            8             //== Invalid File ==================----
#define NPRESULT_INVALIDCALFILE         9             //== Invalid Calibration File ======----
#define NPRESULT_UNABLETOINITIALIZE     10            //== Unable To Initialize ==========----
#define NPRESULT_INVALIDLICENSE         11            //== Invalid License ===============----
#define NPRESULT_NOFRAMEAVAILABLE       14            //== No Frames Available ===========----
#define NPRESULT_DEVICESINUSE           15            //== Another process is currently using the devices

//== CAMERA VIDEO TYPE DEFINITIONS =======================================================----

#define NPVIDEOTYPE_SEGMENT   0
#define NPVIDEOTYPE_GRAYSCALE 1
#define NPVIDEOTYPE_OBJECT    2
#define NPVIDEOTYPE_PRECISION 4
#define NPVIDEOTYPE_MJPEG     6
#define NPVIDEOTYPE_VIDEO     9

//========================================================================================----

#endif // NPTRACKINGTOOLS_H