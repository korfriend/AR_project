//======================================================================================================-----
// Copyright 2012, NaturalPoint Inc.
//======================================================================================================-----
#pragma once

namespace Core
{
    class cIReader;
    class cIWriter;
}

namespace RigidBodySolver
{
    const int kRigidBodyNameMaxLen = 32;
    const int kRigidBodyModelNameMaxLen = 256;
    const float kDefaultMaxMarkerDeflection = 0.004f;

    class cRigidBodySettings
    {
    public:
        // Default constructor code is inlined here so that it can be used in the Motive API.
        cRigidBodySettings()
            : StaticOrientationConstraint( false )
            , StaticConstraintX( 0 )
            , StaticConstraintY( 1 )
            , StaticConstraintZ( 0 )
            , StaticConstraintAngle( 90 )
            , Unique( true )
        {
            MaxFrameRotation = 30;
            MaxFrameTranslation = 0.1f;  // in meters

            UserData = 1;
            ColorR = 0;
            ColorG = 1.0f;
            ColorB = 0;

            DynamicRotationConstraint = false;
            DynamicTranslationConstraint = false;
            DynamicConstraintFrames = 0;

            Smoothing = 0.0;
            PredictionTime = 0.0;

            DisplayUntracked = true;
            DisplayPivot = true;
            DisplayUntrackedMarkers = true;
            DisplayMarkerQuality = false;
            DisplayQuality = true;
            DisplayOrientation = false;

            mModelName[0] = 0;
            ModelValid = false;
            ModelScale = 1.0f;
            ModelAlpha = 1.0f;
            ModelYaw = 0;
            ModelPitch = 0;
            ModelRoll = 0;
            ModelX = 0;
            ModelY = 0;
            ModelZ = 0;
            PivotScale = 1;

            DisplayPositionHistory = false;
            DisplayOrientationHistory = false;
            DisplayHistoryLength = 500;
            DisplayOrientationSpread = 10;
            DisplayOrientationSize = 10;

            // Solver settings
            Enabled = true;
            MaxMarkerDeflection = kDefaultMaxMarkerDeflection;
            MinimumMarkerCount = 3;
            AcquisitionMarkerCount = 3;
            JumpPrevention = true;
            MinimumHitCount = 2;
            Flexibility = 0.5;
            ShareMarkers = false;
            MaxCalculationTime = 5;
            AcquisitionFrames = 1;

            DisplayTracked = true;
            DisplayLabel = false;

            DisplayModelReplace = true;

            TrackingAlgorithmLevel = Algorithm_Auto;

            LocalizeSearch = false;

            ActiveTagID = 0;
            ActiveTagRfChannel = 0;
        }

        void  Save( Core::cIWriter *serial ) const;
        void  Load( Core::cIReader *serial );

        wchar_t  mName     [kRigidBodyNameMaxLen];
        wchar_t  mModelName[kRigidBodyModelNameMaxLen];

        int   UserData;
        float ColorR;
        float ColorG;
        float ColorB;

        float MaxFrameRotation;             //== deprecated
        float MaxFrameTranslation;          //== deprecated

        bool  DynamicRotationConstraint;    //== deprecated
        bool  DynamicTranslationConstraint; //== deprecated
        int   DynamicConstraintFrames;

        double Smoothing;                   // in the range [0,1]
        double PredictionTime;              // the amount of time to predict the solution forward, in fractional frames

        bool   DisplayUntracked;
        bool   DisplayPivot;
        bool   DisplayUntrackedMarkers;
        bool   DisplayMarkerQuality;
        bool   DisplayQuality;
        bool   DisplayTracked;
        bool   DisplayLabel;
        bool   DisplayOrientation;
        bool   DisplayModelReplace;

        bool   ModelValid;
        float  ModelYaw;
        float  ModelPitch;
        float  ModelRoll;
        float  ModelX;
        float  ModelY;
        float  ModelZ;
        float  ModelScale;
        float  ModelAlpha;
        float  PivotScale;

        bool   DisplayPositionHistory;
        bool   DisplayOrientationHistory;
        int    DisplayHistoryLength;
        int    DisplayOrientationSpread;
        int    DisplayOrientationSize;

        //== Static constraint ==--

        bool    StaticOrientationConstraint;
        double  StaticConstraintX;
        double  StaticConstraintY;
        double  StaticConstraintZ;
        double  StaticConstraintAngle;

        // Solver settings
        bool    Enabled;
        float   MaxMarkerDeflection;
        int     MinimumMarkerCount;
        int     AcquisitionMarkerCount;  //== Additional number or markers required for acquisition ==--
        int     MinimumHitCount;         //== deprecated
        float   Flexibility;
        bool    ShareMarkers;
        bool    Unique;
        double  MaxCalculationTime;
        int     AcquisitionFrames;
        bool    JumpPrevention;

        bool    LocalizeSearch;

        enum eTrackingAlgorithms
        {
            Algorithm_Auto = 0,
            Algorithm_MarkerBased,
            Algorithm_RayBased,
            Algorithm_RayBasedPlus,
            Algorithm_RayBasedUltimate,
            Algorithm_Unspecified
        };

        eTrackingAlgorithms TrackingAlgorithmLevel;

        int ActiveTagID;
        int ActiveTagRfChannel;

        int OcclusionModel;  // 0 = Default  1 = Cylinder (Y-Axis)
    };
}
