//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include "ohmd_config.h" // which hmds and trackers to use

#include <openvr_driver.h>
#include "driverlog.h"

#include <assert.h>

#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <sstream>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#include <openhmd.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

using namespace vr;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

ohmd_context* ctx;

class COpenHMDDeviceDriverController;


// gets float values from the device and prints them
void print_infof(ohmd_device* hmd, const char* name, int len, ohmd_float_value val)
{
    float f[20];
    assert (len <= 20);

    ohmd_device_getf(hmd, val, f);
    printf("%-25s", name);
    for(int i = 0; i < len; i++)
        printf("%f ", f[i]);
    printf("\n");
}

// gets int values from the device and prints them
void print_infoi(ohmd_device* hmd, const char* name, int len, ohmd_int_value val)
{
    int iv[20];
    assert (len <= 20);

    ohmd_device_geti(hmd, val, iv);
    printf("%-25s", name);
    for(int i = 0; i < len; i++)
        printf("%d ", iv[i]);
    printf("\n");
}

// keys for use with the settings API
static const char * const k_pch_Sample_Section = "driver_openhmd";
static const char * const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

HmdQuaternion_t identityquat{ 1, 0, 0, 0};

//-----------------------------------------------------------------------------
// What we let SteamVR extrapolate with.
//
// SteamVR does not display the pose we hand it. It dead-reckons that pose
// forward to photon time using vecVelocity, vecAcceleration, vecAngularVelocity
// and poseTimeOffset, and shows the result. So every one of those fields is an
// instruction to move the world, and any error in them is amplified by the
// prediction horizon (~22-33 ms here) before it reaches the eye. Overshoot on a
// hard stop is what that failure looks like.
//
// The evidence that this is where the artifact lives:
//
//   - The same OpenHMD tracking under Monado has no overshoot. Monado's ohmd
//     driver never reports linear velocity at all and ignores at_timestamp_ns
//     (oh_device.c:383), so nothing there can extrapolate; latency is absorbed
//     by the compositor reprojecting onto a freshly *measured* pose.
//   - Monado's own SteamVR driver deliberately zeroes all five of these fields
//     (steamvr_drv/ovrd_driver.cpp:1356-1379, "monado predicts pose 'now'").
//     Somebody else hit this and chose to defeat SteamVR's prediction too.
//   - Our own pose captures said the tracking was clean - but they were logged
//     at prediction horizon 0.0, which is the one horizon at which SteamVR's
//     extrapolation cannot appear. They never covered this.
//
// MEASURED 2026-07-31, and the answer was no. With every term switched off the
// extrapolation is provably total - SteamVR moved the pose by exactly 0.000 mm
// at 11, 22 and 33 ms - and the reported artifact was unchanged. So prediction
// is not the cause.
//
// It is also measurably GOOD, which settles the default. Scored with
// tools/analyze_prediction.py on captures/lin/2026-07-31/predold2_*, while
// moving (|w| > 0.5 rad/s):
//
//   horizon   predicted        no prediction
//    11 ms    1.17 mm 0.146d   4.12 mm 1.013d
//    22 ms    2.57 mm 0.393d   8.23 mm 2.020d
//    33 ms    4.20 mm 0.747d  12.34 mm 3.026d
//
// Three times better than not predicting, and in the same league as the Oculus
// runtime's 0.19 deg / 2.2 mm. Disabling it would be a pure latency regression
// that fixes nothing, so prediction stays on by default and this knob exists
// for experiments only:
//
//   OHMD_STEAMVR_PREDICT unset|1|all               everything (default)
//   OHMD_STEAMVR_PREDICT=0|none                    nothing extrapolated
//   OHMD_STEAMVR_PREDICT=offset,vel,angvel,accel   any subset, comma-separated
//
// Applies to the HMD and to both controllers. The knob it replaces was wired
// only into the HMD path, so an earlier A/B "cleared" prediction while the
// controllers went on extrapolating - one of two reasons that test was void.
//-----------------------------------------------------------------------------
enum PredictTerm {
    kPredictNothing    = 0,
    kPredictTimeOffset = 1 << 0,
    kPredictAngVel     = 1 << 1,
    kPredictVel        = 1 << 2,
    kPredictAccel      = 1 << 3,
    kPredictEverything = kPredictTimeOffset | kPredictAngVel |
                         kPredictVel | kPredictAccel,
};

static int SteamVRPredictTerms()
{
    static int terms = -1;
    if (terms >= 0)
        return terms;

    const char *e = getenv("OHMD_STEAMVR_PREDICT");
    if (e == NULL || *e == '\0') {
        terms = kPredictEverything;
    } else if (strcmp(e, "1") == 0 || strcmp(e, "all") == 0) {
        terms = kPredictEverything;
    } else if (strcmp(e, "0") == 0 || strcmp(e, "none") == 0) {
        terms = kPredictNothing;
    } else {
        terms = kPredictNothing;
        std::string s(e);
        size_t start = 0;
        while (start <= s.size()) {
            size_t comma = s.find(',', start);
            if (comma == std::string::npos)
                comma = s.size();
            std::string tok = s.substr(start, comma - start);
            if (tok == "offset")      terms |= kPredictTimeOffset;
            else if (tok == "angvel") terms |= kPredictAngVel;
            else if (tok == "vel")    terms |= kPredictVel;
            else if (tok == "accel")  terms |= kPredictAccel;
            else if (!tok.empty())
                DriverLog("driver_openhmd: OHMD_STEAMVR_PREDICT: unknown term "
                          "'%s', ignored\n", tok.c_str());
            start = comma + 1;
        }
    }

    DriverLog("driver_openhmd: SteamVR prediction terms = 0x%x "
              "(offset=%d angvel=%d vel=%d accel=%d)\n", terms,
              !!(terms & kPredictTimeOffset), !!(terms & kPredictAngVel),
              !!(terms & kPredictVel), !!(terms & kPredictAccel));
    return terms;
}

// Strip whatever SteamVR is not allowed to extrapolate with. Call once, on the
// fully populated pose, immediately before returning it.
static void ApplyPredictionPolicy(DriverPose_t &pose)
{
    const int terms = SteamVRPredictTerms();

    if (!(terms & kPredictTimeOffset))
        pose.poseTimeOffset = 0.0;
    for (int i = 0; i < 3; i++) {
        if (!(terms & kPredictAngVel))
            pose.vecAngularVelocity[i] = 0.0;
        if (!(terms & kPredictVel))
            pose.vecVelocity[i] = 0.0;
        if (!(terms & kPredictAccel))
            pose.vecAcceleration[i] = 0.0;
        // Never populated by this driver in the first place. Zero it explicitly
        // so a second-order linear extrapolation is never paired with a
        // first-order angular one - that asymmetry is felt as some parts of the
        // scene settling later than others.
        pose.vecAngularAcceleration[i] = 0.0;
    }
}

//-----------------------------------------------------------------------------
// The rendered field of view, which is the gain between head motion and world
// motion.
//
// GetProjectionRaw hands SteamVR four tangents. If those tangents describe a
// NARROWER cone than the lens actually presents to the eye, the world is
// magnified: turning the head by theta sweeps the world across the retina by
// more than theta, so the world over-rotates while you move and only agrees
// with reality once you stop. That artifact is uniform across the image, is
// exactly zero at rest, grows with speed, applies identically to the headset
// and to the controllers because they share the projection, and is completely
// untouchable by anything on the tracking side.
//
// Measured 2026-07-31: the two stacks disagree. From identical OpenHMD
// constants, and inverting this driver's own logged frustum,
//
//   SteamVR-OpenHMD   73.78 deg H   79.73 deg V
//   Monado            79.02 deg H   85.20 deg V
//
// which is exactly a uniform tangent scale of 1.09905. The user reports the
// Monado configuration feels right and this one does not.
//
// The two derivations differ in what they use as the panel-metres-to-tangent
// scale. We use display_info.eye_to_screen_distance (39.62 mm), a physical
// distance. Monado reads OHMD_RIGHT_EYE_FOV, which rift.c computes as TWICE the
// outer half-angle - it treats an asymmetric frustum as symmetric - and then
// back-solves 36.05 mm to make the totals agree.
//
// Oculus keeps these as separate quantities. Rift.dll serialises, per lens,
//
//   LensConfigurations[%d].LensToScreen
//   LensConfigurations[%d].MetersPerTanAngleAtCenter
//   LensConfigurations[%d].PerMMEyeShiftSwim = [7 coefficients]
//
// so the tangent scale is NOT the lens-to-screen distance, and they carry an
// explicit per-lens model of world swim as a function of eye shift. Using a
// physical distance as the tangent scale is therefore very likely the actual
// defect here, and 39.62 mm is very likely the wrong number for this job.
//
//   OHMD_STEAMVR_FOV_SCALE=<f>   multiply all four tangents by f
//   OHMD_STEAMVR_FOV=monado      use Monado's derivation, solved at runtime
//   (unset)                      unchanged - OpenHMD's frustum as-is
//
// Default deliberately changes nothing until the A/B says which way is right.
//-----------------------------------------------------------------------------
static bool FovUseMonado()
{
    static int use = -1;
    if (use < 0) {
        const char *e = getenv("OHMD_STEAMVR_FOV");
        use = (e != NULL && strcmp(e, "monado") == 0) ? 1 : 0;
    }
    return use != 0;
}

static float FovExplicitScale()
{
    static float scale = -1.0f;
    if (scale < 0.0f) {
        const char *e = getenv("OHMD_STEAMVR_FOV_SCALE");
        scale = 1.0f;
        if (e != NULL && *e != '\0') {
            float v = (float)atof(e);
            // A scale outside this range is a typo, not an experiment; a zero
            // or negative one would collapse the frustum and black the headset.
            if (v > 0.5f && v < 2.0f)
                scale = v;
            else
                DriverLog("driver_openhmd: OHMD_STEAMVR_FOV_SCALE=%s out of "
                          "range (0.5..2.0), ignored\n", e);
        }
    }
    return scale;
}

// Reproduce Monado's number rather than hardcoding 1.09905, so this stays
// correct if the panel constants ever change. Monado feeds
// math_compute_fovs() the scalar OHMD fov and it solves for the eye-to-screen
// distance d satisfying atan(inner/d) + atan(outer/d) == fov, where that fov is
// itself 2*atan(outer/near). The whole effect is then a uniform tangent scale
// of near/d, because the panel distances are unchanged.
static float FovMonadoScale(float tan_l, float tan_r, float near_m)
{
    const double outer_t = (fabs(tan_l) > fabs(tan_r)) ? fabs(tan_l) : fabs(tan_r);
    const double inner_t = (fabs(tan_l) > fabs(tan_r)) ? fabs(tan_r) : fabs(tan_l);
    const double outer = outer_t * near_m;
    const double inner = inner_t * near_m;
    const double target = 2.0 * atan(outer / near_m);

    double lo = 1e-4, hi = 1.0;
    for (int i = 0; i < 200; i++) {
        const double d = 0.5 * (lo + hi);
        if (atan(inner / d) + atan(outer / d) > target)
            lo = d;
        else
            hi = d;
    }
    const double d = 0.5 * (lo + hi);
    return (d > 1e-6) ? (float)(near_m / d) : 1.0f;
}

// Apply whichever policy is selected, and state the result in degrees. The log
// line matters: two experiments earlier in this project were void because the
// build under test was never the build that ran, and there was no way to tell
// from the log which one had. An A/B that cannot be verified from its own
// output is not a measurement.
static void ApplyFovPolicy(const char *eye, float *l, float *r, float *t,
                           float *b, float near_m)
{
    float scale = FovExplicitScale();
    const char *how = "openhmd";
    if (FovUseMonado()) {
        scale = FovMonadoScale(*l, *r, near_m);
        how = "monado";
    } else if (scale != 1.0f) {
        how = "scaled";
    }

    *l *= scale; *r *= scale; *t *= scale; *b *= scale;

    const double h = (atan(fabs((double)*l)) + atan(fabs((double)*r))) * 180.0 / M_PI;
    const double v = (atan(fabs((double)*t)) + atan(fabs((double)*b))) * 180.0 / M_PI;
    DriverLog("driver_openhmd: FOV %-5s mode=%-7s scale=%.5f -> H %.2f deg  V %.2f deg\n",
              eye, how, scale, h, v);
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

class CWatchdogDriver_OpenHMD : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_OpenHMD()
    {
        DriverLog("Created watchdog object\n");
        m_pWatchdogThread = nullptr;
    }
    virtual ~CWatchdogDriver_OpenHMD() {}

    virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
    virtual void Cleanup() ;

private:
	std::thread *m_pWatchdogThread;
};

CWatchdogDriver_OpenHMD g_watchdogDriverOpenHMD;


bool g_bExiting = false;

void WatchdogThreadFunction(  )
{
    while ( !g_bExiting )
    {
#if defined( _WINDOWS )
        // on windows send the event when the Y key is pressed.
        if ( (0x01 & GetAsyncKeyState( 'Y' )) != 0 )
        {
            // Y key was pressed.
            vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
        }
        std::this_thread::sleep_for( std::chrono::microseconds( 500 ) );
#else
        DriverLog("Watchdog wakeup\n");
        // for the other platforms, just send one every five seconds
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
#endif
    }
    DriverLog("Watchdog exit\n");
}

EVRInitError CWatchdogDriver_OpenHMD::Init( vr::IVRDriverContext *pDriverContext )
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT( pDriverContext );
    InitDriverLog( vr::VRDriverLog() );

    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to
    // be pressed. A real driver should wait for a system button event or something else from the
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    
    DriverLog("starting watchdog thread\n");

    m_pWatchdogThread = new std::thread( WatchdogThreadFunction );
    if ( !m_pWatchdogThread )
    {
        DriverLog( "Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
    }


    void CWatchdogDriver_OpenHMD::Cleanup()
    {
    g_bExiting = true;
    if ( m_pWatchdogThread )
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}

class COpenHMDDeviceDriverController : public vr::ITrackedDeviceServerDriver /*, public vr::IVRControllerComponent */ {
public:
    int index;
    ohmd_device* device;
    int device_idx;
    int device_flags;
    DriverPose_t pose;
    double untracked_pos[3] = { 0.0, 0.0, 0.0 };

    bool m_is_oculus;

    COpenHMDDeviceDriverController(int index, ohmd_device* _device, int _device_idx) :
		index(index), device(_device), device_idx(_device_idx) {
        DriverLog("construct controller object %d (OpenHMD device %d)\n", index, device_idx);
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        pose = { 0 };


        if (strcmp(ohmd_list_gets(ctx, device_idx, OHMD_VENDOR), "Oculus VR, Inc.") == 0) {
            m_is_oculus = true;
            DriverLog("detected oculus controllers, using oculus input profile");
        }
    }
    virtual ~COpenHMDDeviceDriverController() {}

    EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        DriverLog("activate controller %d: %d\n", index, unObjectId);
        m_unObjectId = unObjectId;

        if (this->device == NULL) {
          return VRInitError_Init_InterfaceNotFound;
        }

        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

        const char *controllerModel = ohmd_list_gets(ctx, device_idx, OHMD_PRODUCT);

	ohmd_list_geti(ctx, device_idx, OHMD_DEVICE_FLAGS, &device_flags);

        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, controllerModel);
        //vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, controllerModel);
	//vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, "1" );
        if (m_is_oculus) {
            // steamapps/common/SteamVR/drivers/oculus/resources/input/touch_profile.json -> "controller_type": "oculus_touch"
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "oculus_touch" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json" );

            if (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) {
                // steamapps/common/SteamVR/resources/rendermodels/oculus_cv1_controller_left
                vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
            } else if (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER) {
                // steamapps/common/SteamVR/resources/rendermodels/oculus_cv1_controller_right
                vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
            }
        } else {
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "openhmd_controller" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{openhmd}/input/openhmd_controller_profile.json" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "vr_controller_vive_1_5");
        }


        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

	// The resting position used when the controller has no positional
	// tracking - down and to the side, so it appears where a hand would
	// plausibly be rather than inside the user's head at the origin.
	// This used to be written into `pose` here, but GetPose() starts with
	// `pose = { 0 }`, so it was erased before it was ever reported. Keep it
	// as its own value and let GetPose() apply it when it is actually needed.
	untracked_pos[0] = (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) ? -0.25 : 0.25;
	untracked_pos[1] = -0.5;
	untracked_pos[2] = 0.15;

	if (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) {
           DriverLog("Left Controller\n");
           vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
	} else {
           DriverLog("Right Controller\n");
           vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
	}


        int control_count;
        ohmd_device_geti(device, OHMD_CONTROL_COUNT, &control_count);
        if (control_count > 64)
          control_count = 64;

        const char* controls_fn_str[] = { "generic", "trigger", "trigger_click", "squeeze", "menu", "home",
                "analog-x", "analog-y", "anlog_press", "button-a", "button-b", "button-x", "button-y",
                "volume-up", "volume-down", "mic-mute"};
        const char* controls_type_str[] = {"digital", "analog"};

        int controls_fn[64];
        int controls_types[64];

        ohmd_device_geti(device, OHMD_CONTROLS_HINTS, controls_fn);
        ohmd_device_geti(device, OHMD_CONTROLS_TYPES, controls_types);

        for(int i = 0; i < control_count; i++){
          DriverLog("%s (%s)%s\n", controls_fn_str[controls_fn[i]], controls_type_str[controls_types[i]], i == control_count - 1 ? "" : ", ");
          const char *control_map = NULL, *touch_map = NULL;
          EVRScalarUnits analog_type = VRScalarUnits_NormalizedOneSided;

          m_buttons[i] = k_ulInvalidInputComponentHandle;
          m_analogControls[i] = k_ulInvalidInputComponentHandle;
          m_touchControls[i] = k_ulInvalidInputComponentHandle;

          // TODO: inputs match steamapps/common/SteamVR/drivers/oculus/resources/input/touch_profile.json
          // but also support other controllers
          switch (controls_fn[i]) {
            case OHMD_GENERIC:
              control_map = "/input/generic/click";
              break;
            case OHMD_TRIGGER:
              control_map = "/input/trigger/value";
              break;
            case OHMD_TRIGGER_CLICK:
              control_map = "/input/trigger/click";
              break;
            case OHMD_SQUEEZE:
              control_map = "/input/grip/value";
              break;
            case OHMD_MENU:
              control_map = "/input/system/click";
              break;
            case OHMD_HOME:
                // TODO: this button doesn't have an input in touch_profile.json
              control_map = "/input/system/click";
              break;
            case OHMD_ANALOG_X:
              control_map = "/input/joystick/x";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case OHMD_ANALOG_Y:
              control_map = "/input/joystick/y";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case OHMD_ANALOG_PRESS:
              control_map = "/input/joystick/click";
              break;
            case OHMD_BUTTON_A:
              control_map = "/input/a/click";
              touch_map = "/input/a/touch";
              break;
            case OHMD_BUTTON_B:
              control_map = "/input/b/click";
              touch_map = "/input/b/touch";
              break;
            case OHMD_BUTTON_X:
              control_map = "/input/x/click";
              touch_map = "/input/x/touch";
              break;
            case OHMD_BUTTON_Y:
              control_map = "/input/y/click";
              touch_map = "/input/y/touch";
              break;

            default:
              break;
          }

          /* We fall through here for generic buttons */
          if (control_map != NULL) {
            if (controls_types[i] == OHMD_DIGITAL) {
              vr::VRDriverInput()->CreateBooleanComponent( m_ulPropertyContainer, control_map, m_buttons + i);
            }
            else {
              vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, control_map, m_analogControls + i, VRScalarType_Absolute, analog_type);
            }
          }
	  if (touch_map != NULL) {
              vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, touch_map, m_touchControls + i, VRScalarType_Absolute, analog_type);
          }
        }

        return VRInitError_None;
    }

    void Deactivate()
    {
        DriverLog("deactivate controller\n");
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby()
    {
                DriverLog("standby controller\n");
    }

    void *GetComponent( const char *pchComponentNameAndVersion )
    {
        DriverLog("get controller component %s | %s ", pchComponentNameAndVersion, /*vr::IVRControllerComponent_Version*/ "<nothing>");
        if (!strcmp(pchComponentNameAndVersion, /*vr::IVRControllerComponent_Version*/ "<nothing>"))
        {
            DriverLog(": yes\n");
            return NULL;//(vr::IVRControllerComponent*)this;

        }

        DriverLog(": no\n");
        return NULL;
    }

    /** debug request from a client */
    void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
    {
        if( unResponseBufferSize >= 1 )
            pchResponseBuffer[0] = 0;
    }

    DriverPose_t GetPose()
    {
	pose = { 0 };
	pose.deviceIsConnected = true;

	// Report what we actually have. This used to claim poseIsValid with
	// TrackingResult_Running_OK unconditionally, so a controller with no
	// tracking at all was presented as confidently located at the origin -
	// inside the user's head - rather than as untracked.
	const bool has_rot = (device_flags & OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING) != 0;
	const bool has_pos = (device_flags & OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING) != 0;
	pose.poseIsValid = has_rot || has_pos;
	pose.result = pose.poseIsValid ? TrackingResult_Running_OK
	                               : TrackingResult_Uninitialized;
	if (!has_pos) {
		// Orientation-only: park it where a hand plausibly is.
		for (int i = 0; i < 3; i++)
			pose.vecPosition[i] = untracked_pos[i];
	}

	if (device_flags & OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING) {
		float quat[4];
		ohmd_device_getf(device, OHMD_ROTATION_QUAT, quat);
		pose.qRotation.x = quat[0];
		pose.qRotation.y = quat[1];
		pose.qRotation.z = quat[2];
		pose.qRotation.w = quat[3];

		float angvel[3];
		ohmd_device_getf(device, OHMD_ANGULAR_VELOCITY_VECTOR, angvel);
		pose.vecAngularVelocity[0] = angvel[0];
		pose.vecAngularVelocity[1] = angvel[1];
		pose.vecAngularVelocity[2] = angvel[2];
	}

	if (device_flags & OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING) {
		float pos[3];
		ohmd_device_getf(device, OHMD_POSITION_VECTOR, pos);
		pose.vecPosition[0] = pos[0];
		pose.vecPosition[1] = pos[1];
		pose.vecPosition[2] = pos[2];

		float vel[3];
		ohmd_device_getf(device, OHMD_VELOCITY_VECTOR, vel);
		pose.vecVelocity[0] = vel[0];
		pose.vecVelocity[1] = vel[1];
		pose.vecVelocity[2] = vel[2];

		// Acceleration makes SteamVR's extrapolation second-order instead of
		// purely linear. OpenHMD has always exposed it; we simply never read it.
		float accel[3];
		ohmd_device_getf(device, OHMD_ACCELERATION_VECTOR, accel);
		pose.vecAcceleration[0] = accel[0];
		pose.vecAcceleration[1] = accel[1];
		pose.vecAcceleration[2] = accel[2];
	}

	// SteamVR predicts this pose forward to photon time itself, using the
	// velocities above and poseTimeOffset as the epoch. Leaving the offset at 0
	// claims the pose was sampled at the instant of this call, which is a lie by
	// the pipeline latency (USB + fusion), so SteamVR under-predicts by exactly
	// that much. Report the real age instead; it is negative because the sample
	// is in the past.
	float pose_age = 0.0f;
	ohmd_device_getf(device, OHMD_POSE_AGE_SECONDS, &pose_age);
	pose.poseTimeOffset = -pose_age;

	// DriverLog("get controller %d pose %f %f %f %f, %f %f %f\n", index, quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

	for (int i = 0; i < 3; i++) {
		pose.vecDriverFromHeadTranslation[i] = 0.0;
		pose.vecWorldFromDriverTranslation[i] = 0.0;
	}

	if (m_is_oculus) {
		// Where the Touch actually is, rather than where upstream guessed.
		//
		// OpenHMD reports the pose of the LED-model frame (rift.c registers the
		// controller with an identity model_pose), but SteamVR draws
		// oculus_cv1_controller_left/right - and derives the OpenXR grip and aim
		// poses - in Valve's render-model frame. driverFromHead is the transform
		// between them, i.e. device <- rendermodel.
		//
		// Both frames are pinned by authoritative data we already have, so this
		// is measured rather than tuned: Oculus's own 24-LED model, read out of
		// the controller's flash, sits physically on the ring that Valve's mesh
		// also models. Registering the LED cloud onto the mesh (rigid ICP, both
		// controllers solved independently) converges to mirror-image transforms
		// with a 2.1 mm mean residual - the depth the LEDs sit below the shell.
		// Mapping the JSON's landmarks back through it lands openxr_aim on the
		// ring's outward side and openxr_grip/base on the hand side, which is
		// the arrangement the hardware has.
		//
		// The old constants put the controller 129 mm from where it belongs:
		// they pushed +80 mm along Z when the correct offset pulls 44.5 mm the
		// other way. The rotation was close (30 deg vs 33.06 deg measured),
		// which is consistent with having been eyeballed until it looked right.
		//
		// OHMD_TOUCH_LEGACY_OFFSET=1 restores the old values for comparison.
		static const bool legacy_offset =
			getenv("OHMD_TOUCH_LEGACY_OFFSET") != NULL;

		if (legacy_offset) {
			const HmdQuaternion_t oculusOffsetQ = { 0.966, 0.259, 0, 0};
			pose.qDriverFromHeadRotation = oculusOffsetQ;
			pose.vecDriverFromHeadTranslation[2] = 0.08;
		} else {
			// +33.06 deg about X
			const HmdQuaternion_t touchOffsetQ = { 0.958669, 0.284523, 0, 0 };
			pose.qDriverFromHeadRotation = touchOffsetQ;

			// mirror-symmetric in X, as the two controllers are
			bool left = (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) != 0;
			pose.vecDriverFromHeadTranslation[0] = left ? -0.00813 : 0.00813;
			pose.vecDriverFromHeadTranslation[1] = 0.03201;
			pose.vecDriverFromHeadTranslation[2] = -0.04453;
		}

		pose.qWorldFromDriverRotation = identityquat;
	}
	else {
		pose.qDriverFromHeadRotation = identityquat;
		pose.qWorldFromDriverRotation = identityquat;
	}

	ApplyPredictionPolicy(pose);
	return pose;
    }

    void RunFrame() {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof( DriverPose_t ) );

        int control_count;
        float control_state[256];

        ohmd_device_geti(device, OHMD_CONTROL_COUNT, &control_count);
        if (control_count > 64)
          control_count = 64;

        ohmd_device_getf(device, OHMD_CONTROLS_STATE, control_state);

        for (int i = 0; i < control_count; i++) {
          if (m_buttons[i] != k_ulInvalidInputComponentHandle) {
            vr::VRDriverInput()->UpdateBooleanComponent( m_buttons[i], control_state[i] != 0, 0 );
          }
          else if (m_analogControls[i] != k_ulInvalidInputComponentHandle)
            vr::VRDriverInput()->UpdateScalarComponent( m_analogControls[i], control_state[i], 0 );
	  /* If the control is not 0, mark it touched */
          if (m_touchControls[i] != k_ulInvalidInputComponentHandle) {
            vr::VRDriverInput()->UpdateScalarComponent( m_touchControls[i], control_state[i] == 0 ? 0.0 : 1.0, 0 );
          }
        }
    }

    VRControllerState_t controllerstate;

    VRControllerState_t GetControllerState() {
    DriverLog("get controller state\n");
    //return controllerstate;

    controllerstate.unPacketNum = controllerstate.unPacketNum + 1;
    //TODO: buttons

    //TODO: nolo says when a button was pressed a button was also touched. is that so?
    controllerstate.ulButtonTouched |= controllerstate.ulButtonPressed;

    //uint64_t ulChangedTouched = controllerstate.ulButtonTouched ^ controllerstate.ulButtonTouched;
    //uint64_t ulChangedPressed = controllerstate.ulButtonPressed ^ controllerstate.ulButtonPressed;
    return controllerstate;
    }

    bool TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds ) {
        return false;
    }

    std::string GetSerialNumber() const { 
        DriverLog("get controller serial number %s\n", m_sSerialNumber.c_str());
        return m_sSerialNumber;
    }

private:
    std::string m_sSerialNumber = "Controller serial number " + std::to_string(index);
    std::string m_sModelNumber = "Controller model number " + std::to_string(index);
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    /* Generic button controls */
    vr::VRInputComponentHandle_t m_buttons[64]; /* Maximum components we support */
    /* Analog controls */
    vr::VRInputComponentHandle_t m_analogControls[64]; /* Maximum components we support */
    /* Touch controls */
    vr::VRInputComponentHandle_t m_touchControls[64]; /* Maximum components we support */
};

class COpenHMDDeviceDriver final : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
    COpenHMDDeviceDriver(int hmddisplay_idx, int hmdtracker_idx)
    {
        hmd = ohmd_list_open_device(ctx, hmddisplay_idx);
        if (hmdtracker_idx != -1 && hmdtracker_idx != hmddisplay_idx)
	    hmdtracker = ohmd_list_open_device(ctx, hmdtracker_idx);
	else
	    hmdtracker = NULL;

        if(!hmd){
            DriverLog("failed to open device: %s\n", ohmd_ctx_get_error(ctx));
        }

        int ivals[2];
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, ivals);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, ivals + 1);
        //DriverLog("resolution:              %i x %i\n", ivals[0], ivals[1]);

        /*
        print_infof(hmd, "hsize:",            1, OHMD_SCREEN_HORIZONTAL_SIZE);
        print_infof(hmd, "vsize:",            1, OHMD_SCREEN_VERTICAL_SIZE);
        print_infof(hmd, "lens separation:",  1, OHMD_LENS_HORIZONTAL_SEPARATION);
        print_infof(hmd, "lens vcenter:",     1, OHMD_LENS_VERTICAL_POSITION);
        print_infof(hmd, "left eye fov:",     1, OHMD_LEFT_EYE_FOV);
        print_infof(hmd, "right eye fov:",    1, OHMD_RIGHT_EYE_FOV);
        print_infof(hmd, "left eye aspect:",  1, OHMD_LEFT_EYE_ASPECT_RATIO);
        print_infof(hmd, "right eye aspect:", 1, OHMD_RIGHT_EYE_ASPECT_RATIO);
        print_infof(hmd, "distortion k:",     6, OHMD_DISTORTION_K);

        print_infoi(hmd, "digital button count:", 1, OHMD_BUTTON_COUNT);
        */

        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        DriverLog( "Using settings values\n" );
        ohmd_device_getf(hmd, OHMD_EYE_IPD, &m_flIPD);
        
        {
            std::stringstream buf;
            buf << ohmd_list_gets(ctx, hmddisplay_idx, OHMD_PRODUCT);
            buf << ": ";
            buf << ohmd_list_gets(ctx, hmddisplay_idx, OHMD_PATH);
            m_sSerialNumber = buf.str();
        }

        {
            std::stringstream buf;
            buf << "OpenHMD: ";
            buf << ohmd_list_gets(ctx, hmddisplay_idx, OHMD_PRODUCT);
            m_sModelNumber = buf.str();
        }

        // Important to pass vendor through. Gaze cursor is only available for "Oculus". So grab the first word.
        char const* vendor_override = getenv("OHMD_VENDOR_OVERRIDE");
        if (vendor_override) {
            m_sVendor = vendor_override;
        } else {
            m_sVendor = ohmd_list_gets(ctx, hmddisplay_idx, OHMD_VENDOR);
            if (m_sVendor.find(' ') != std::string::npos) {
                m_sVendor = m_sVendor.substr(0, m_sVendor.find(' '));
            }
        }

        m_nWindowX = 1920; //TODO: real window offset
        m_nWindowY = 0;
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &m_nWindowWidth);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, &m_nWindowHeight );
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &m_nRenderWidth);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, &m_nRenderHeight );
        //m_nRenderWidth /= 2;
        //m_nRenderHeight /= 2;

        m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );

        // Prop_DisplayFrequency_Float is not decoration: SteamVR derives the
        // frame period from it, and the frame period is what sets how far ahead
        // the compositor predicts every pose it draws. Shipped, this read
        // returned 0 - resources/settings/default.vrsettings carries
        // "displayFrequency": 0 and nothing ever overrode it, so the log said
        // "Display Frequency: 0.000000" on every run since the driver existed.
        //
        // A zero frame period makes the photon-time arithmetic meaningless,
        // which is a motion-proportional error: invisible at rest, growing with
        // speed, and applied identically to the HMD and to both controllers
        // because the compositor predicts them all against the same display
        // clock. That profile survived seven fixes on the tracking side, none
        // of which could touch it - the pose was fine, what the compositor did
        // with it was not. It also explains why disabling prediction outright
        // (OHMD_STEAMVR_NO_PREDICT=1) felt better despite adding latency.
        //
        // OpenHMD exposes no refresh-rate property, which is why upstream left
        // a "TODO: find actual frequency somehow" here. The CV1's panel is
        // 2160x1200 at 90 Hz - fixed, not a mode the host chooses - so use that
        // and let the setting override for other hardware.
        m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float );
        if ( m_flDisplayFrequency <= 0.0f )
        {
            m_flDisplayFrequency = 90.0f;
            DriverLog( "driver_openhmd: displayFrequency was %f; using %.1f Hz. "
                "SteamVR predicts poses over one frame period, so a zero here "
                "makes its photon-time arithmetic meaningless.\n",
                vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float ),
                m_flDisplayFrequency );
        }

        DriverLog( "driver_openhmd: Vendor: %s\n", m_sVendor.c_str() );
        DriverLog( "driver_openhmd: Serial Number: %s\n", m_sSerialNumber.c_str() );
        DriverLog( "driver_openhmd: Model Number: %s\n", m_sModelNumber.c_str() );
        DriverLog( "driver_openhmd: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
        DriverLog( "driver_openhmd: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
        DriverLog( "driver_openhmd: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
                                                DriverLog( "driver_openhmd: Display Frequency: %f\n", m_flDisplayFrequency );
        DriverLog( "driver_openhmd: IPD: %f\n", m_flIPD );

        float distortion_coeffs[4];
        ohmd_device_getf(hmd, OHMD_UNIVERSAL_DISTORTION_K, &(distortion_coeffs[0]));
        DriverLog("driver_openhmd: Distortion values a=%f b=%f c=%f d=%f\n", distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3]);

	/* Sleep for 1 second while activating to let the display connect */
	std::this_thread::sleep_for( std::chrono::seconds(1) );
    }

    virtual ~COpenHMDDeviceDriver()
    {
    }


    EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ManufacturerName_String, m_sVendor.c_str());
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );

        //float sep;

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        return VRInitError_None;
    }

    void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby()
    {
    }

    void *GetComponent( const char *pchComponentNameAndVersion )
    {
        if ( !strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) )
        {
            return (vr::IVRDisplayComponent*)this;
        }
        return NULL;
    }

    void PowerOff()
    {
    }

    void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
    {
        if( unResponseBufferSize >= 1 )
            pchResponseBuffer[0] = 0;
    }

    void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnX = m_nWindowX;
        *pnY = m_nWindowY;
        *pnWidth = m_nWindowWidth;
        *pnHeight = m_nWindowHeight;
    }

    bool IsDisplayOnDesktop()
    {
        /* A CV1 is not part of the desktop and never can be: the kernel marks
         * its connector non-desktop=1, so the compositor deliberately refuses
         * to lay it out, and there is no desktop rectangle for it to occupy.
         * Claiming otherwise contradicts Prop_IsOnDesktop_Bool, which this
         * driver already sets to false, and sends SteamVR looking for a window
         * at the hardcoded (1920, 0) above - coordinates that belong to
         * whatever real monitor happens to be there.
         *
         * Returning false puts SteamVR in direct mode, where it acquires the
         * headset's connector itself. That is the same path Monado takes here
         * successfully (VK_EXT_acquire_drm_display over a wp_drm_lease_v1
         * connector).
         *
         * Set OHMD_STEAMVR_EXTENDED=1 for a headset that genuinely is an
         * extended-desktop display, as the older OpenHMD-supported HMDs are. */
        static int extended = -1;
        if (extended == -1) {
            const char *e = getenv("OHMD_STEAMVR_EXTENDED");
            extended = (e && e[0] == '1');
        }
        return extended != 0;
    }

    bool IsDisplayRealDisplay()
    {
        return true;
    }

    void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnWidth = m_nRenderWidth;
        *pnHeight = m_nRenderHeight;
    }

    void GetEyeOutputViewport( EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnY = 0;
        *pnWidth = m_nWindowWidth / 2;
        *pnHeight = m_nWindowHeight;

        if ( eEye == Eye_Left )
        {
            *pnX = 0;
        }
        else
        {
            *pnX = m_nWindowWidth / 2;
        }
    }

    // flatten 2D indices in a 4x4 matrix explicit so it's easy to see what's happening:
    int f1(int i, int j)
    {
        if (i == 0 && j == 0) return 0;
        if (i == 0 && j == 1) return 1;
        if (i == 0 && j == 2) return 2;
        if (i == 0 && j == 3) return 3;

        if (i == 1 && j == 0) return 4;
        if (i == 1 && j == 1) return 5;
        if (i == 1 && j == 2) return 6;
        if (i == 1 && j == 3) return 7;

        if (i == 2 && j == 0) return 8;
        if (i == 2 && j == 1) return 9;
        if (i == 2 && j == 2) return 10;
        if (i == 2 && j == 3) return 11;

        if (i == 3 && j == 0) return 12;
        if (i == 3 && j == 1) return 13;
        if (i == 3 && j == 2) return 14;
        if (i == 3 && j == 3) return 15;

        return -1;
    }

    int f2(int i, int j)
    {
        if (i == 0 && j == 0) return 0;
        if (i == 0 && j == 1) return 4;
        if (i == 0 && j == 2) return 8;
        if (i == 0 && j == 3) return 12;

        if (i == 1 && j == 0) return 1;
        if (i == 1 && j == 1) return 5;
        if (i == 1 && j == 2) return 9;
        if (i == 1 && j == 3) return 13;

        if (i == 2 && j == 0) return 2;
        if (i == 2 && j == 1) return 6;
        if (i == 2 && j == 2) return 10;
        if (i == 2 && j == 3) return 14;

        if (i == 3 && j == 0) return 3;
        if (i == 3 && j == 1) return 7;
        if (i == 3 && j == 2) return 11;
        if (i == 3 && j == 3) return 15;

        return -1;
    }

    void columnMatrixToAngles(float *yaw, float *pitch, float *roll, float colMatrix[4][4] ) {
        double sinPitch, cosPitch, sinRoll, cosRoll, sinYaw, cosYaw;

        sinPitch = -colMatrix[2][0];
        cosPitch = sqrt(1 - sinPitch*sinPitch);

        if ( abs(cosPitch) > 0.1) 
        {
            sinRoll = colMatrix[2][1] / cosPitch;
            cosRoll = colMatrix[2][2] / cosPitch;
            sinYaw = colMatrix[1][0] / cosPitch;
            cosYaw = colMatrix[0][0] / cosPitch;
        } 
        else 
        {
            sinRoll = -colMatrix[1][2];
            cosRoll = colMatrix[1][1];
            sinYaw = 0;
            cosYaw = 1;
        }

        *yaw   = atan2(sinYaw, cosYaw) * 180 / M_PI;
        *pitch = atan2(sinPitch, cosPitch) * 180 / M_PI;
        *roll  = atan2(sinRoll, cosRoll) * 180 / M_PI;
    } 
    
    typedef union {
        float m[4][4];
        float arr[16];
    } mat4x4f;

    void omat4x4f_mult(const mat4x4f* l, const mat4x4f* r, mat4x4f *o) {
        for(int i = 0; i < 4; i++){
            float a0 = l->m[i][0], a1 = l->m[i][1], a2 = l->m[i][2], a3 = l->m[i][3];
            o->m[i][0] = a0 * r->m[0][0] + a1 * r->m[1][0] + a2 * r->m[2][0] + a3 * r->m[3][0];
            o->m[i][1] = a0 * r->m[0][1] + a1 * r->m[1][1] + a2 * r->m[2][1] + a3 * r->m[3][1];
            o->m[i][2] = a0 * r->m[0][2] + a1 * r->m[1][2] + a2 * r->m[2][2] + a3 * r->m[3][2];
            o->m[i][3] = a0 * r->m[0][3] + a1 * r->m[1][3] + a2 * r->m[2][3] + a3 * r->m[3][3];
        }
    }
    
    void createUnRotation(float angle, mat4x4f *m) {
        memset(m, 0, sizeof(*m));
        m->m[0][0] = 1.0f;
        m->m[1][1] = 1.0f;
        m->m[2][2] = 1.0f;
        m->m[3][3] = 1.0f;
        
        DriverLog("Unrotating for angle %f\n", angle);
        
        if (angle > -5 && angle < 5) {
            return;
        }
        
        else if (angle > 85 && angle < 95) {
            m->m[0][0] = 0.0f; m->m[0][1] = -1.0f; m->m[1][0] = 1.0f; m->m[1][1] = 0.0f;
            m->m[0][1] = 1.0f; m->m[1][0] = -1.0f;
        }

        else if (angle > -95 && angle < -85) {
            m->m[0][0] = 0.0f; m->m[0][1] = -1.0f; m->m[1][0] = 1.0f; m->m[1][1] = 0.0f;
        }
        
        else {
            DriverLog("UNIMPLEMENTED ROTATION!!!\n");
        }
    }

    void GetProjectionRaw( EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
    {
        mat4x4f ohmdprojection;
        if (eEye == Eye_Left) {
            ohmd_device_getf(hmd, OHMD_LEFT_EYE_GL_PROJECTION_MATRIX, ohmdprojection.arr);
        } else {
            ohmd_device_getf(hmd, OHMD_RIGHT_EYE_GL_PROJECTION_MATRIX, ohmdprojection.arr);
        }

        float yaw, pitch, roll;
        float p[4][4];
        memcpy(p, ohmdprojection.arr, 16 * sizeof(float));
        columnMatrixToAngles(&yaw, &pitch, &roll, p);
        
        if (eEye == Eye_Left) {
            rotation_left = yaw;
        } else {
            rotation_right = yaw;
        }
        
        mat4x4f unrotation;
        createUnRotation(yaw, &unrotation);
 
        DriverLog("unrotation\n%f %f %f %f\n%f %f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
            unrotation.arr[0], unrotation.arr[1], unrotation.arr[2], unrotation.arr[3],
            unrotation.arr[4], unrotation.arr[5], unrotation.arr[6], unrotation.arr[7],
            unrotation.arr[8], unrotation.arr[9], unrotation.arr[10], unrotation.arr[11],
            unrotation.arr[12], unrotation.arr[13], unrotation.arr[14], unrotation.arr[15]);
            
        omat4x4f_mult(&ohmdprojection, &unrotation, &ohmdprojection);
        
        // http://stackoverflow.com/questions/10830293/ddg#12926655
        // get projection matrix from openhmd, convert it into lrtb + near,far with SO formula
        // then divide by near plane distance to get the tangents of the angles from the center plane (tan = opposite side = these values divided by adjacent side = near plane distance)
        // but negate top and bottom. who knows why. there are 3 or so issues for it on github

        // f2 switches row-major and column-major
        float m00 = ohmdprojection.arr[f2(0,0)];
        //float m03 = ohmdprojection[f2(0,3)];
        //float m10 = ohmdprojection[f2(1,3)];
        float m11 = ohmdprojection.arr[f2(1,1)];
        //float m13 = ohmdprojection[f2(1,3)];
        float m23 = ohmdprojection.arr[f2(2,3)];
        float m22 = ohmdprojection.arr[f2(2,2)];
        float m12 = ohmdprojection.arr[f2(1,2)];
        float m02 = ohmdprojection.arr[f2(0,2)];

        float near   = m23/(m22-1);
        float far    = m23/(m22+1);
        *pfBottom = -   (m12-1)/m11;
        *pfTop    = -   (m12+1)/m11;
        *pfLeft   =     (m02-1)/m00;
        *pfRight  =     (m02+1)/m00;
        
        DriverLog("m 00 %f, 11 %f, 22 %f, 12 %f, 02 %f\n", m00, m11, m23, m22, m12, m02);

        DriverLog("ohmd projection\n%f %f %f %f\n%f %f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
            ohmdprojection.arr[0], ohmdprojection.arr[1], ohmdprojection.arr[2], ohmdprojection.arr[3],
            ohmdprojection.arr[4], ohmdprojection.arr[5], ohmdprojection.arr[6], ohmdprojection.arr[7],
            ohmdprojection.arr[8], ohmdprojection.arr[9], ohmdprojection.arr[10], ohmdprojection.arr[11],
            ohmdprojection.arr[12], ohmdprojection.arr[13], ohmdprojection.arr[14], ohmdprojection.arr[15]
        );
        
        DriverLog("projectionraw values lrtb, near far: %f %f %f %f | %f %f\n", *pfLeft, *pfRight, *pfTop, *pfBottom, near, far);

        ApplyFovPolicy(eEye == Eye_Left ? "left" : "right",
                       pfLeft, pfRight, pfTop, pfBottom, near);
        
        //DriverLog("angles %f %f %f\n", yaw, pitch, roll);
    }

    DistortionCoordinates_t ComputeDistortion( EVREye eEye, float fU, float fV )
    {
        float angle = (eEye == Eye_Left ? rotation_left : rotation_right);
        
        //DriverLog("Eye %d before: %f %f\n", eEye, fU, fV);
                
        if (angle > -5 && angle < 5) {
        } else if (angle > 85 && angle < 95) {
            float tmp = fV;
            fV = 1. - fU;
            fU = tmp;
        } else if (angle > -95 && angle < -85) {
            float tmp = fV;
            fV = fU;
            fU = 1.f - tmp;
        } else {
            float x = 0 * fU + -1 * fV;
            float y = -1 * fU + 0 * 0 * fV;
            fU = x;
            fV = y;
        }
        
        //DriverLog("Eye %d after: %f %f\n", eEye, fU, fV);
        
        int hmd_w;
        int hmd_h;
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &hmd_w);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, &hmd_h);
        float ipd;
        ohmd_device_getf(hmd, OHMD_EYE_IPD, &ipd);
        float viewport_scale[2];
        float distortion_coeffs[4];
        float aberr_scale[3];
        float sep;
        float left_lens_center[2];
        float right_lens_center[2];
        //viewport is half the screen
        ohmd_device_getf(hmd, OHMD_SCREEN_HORIZONTAL_SIZE, &(viewport_scale[0]));
        viewport_scale[0] /= 2.0f;
        ohmd_device_getf(hmd, OHMD_SCREEN_VERTICAL_SIZE, &(viewport_scale[1]));
        //distortion coefficients
        ohmd_device_getf(hmd, OHMD_UNIVERSAL_DISTORTION_K, &(distortion_coeffs[0]));
        ohmd_device_getf(hmd, OHMD_UNIVERSAL_ABERRATION_K, &(aberr_scale[0]));
        //calculate lens centers (assuming the eye separation is the distance betweenteh lense centers)
        ohmd_device_getf(hmd, OHMD_LENS_HORIZONTAL_SEPARATION, &sep);
        ohmd_device_getf(hmd, OHMD_LENS_VERTICAL_POSITION, &(left_lens_center[1]));
        ohmd_device_getf(hmd, OHMD_LENS_VERTICAL_POSITION, &(right_lens_center[1]));
        left_lens_center[0] = viewport_scale[0] - sep/2.0f;
        right_lens_center[0] = sep/2.0f;
        //asume calibration was for lens view to which ever edge of screen is further away from lens center
        float warp_scale = (left_lens_center[0] > right_lens_center[0]) ? left_lens_center[0] : right_lens_center[0];

        float lens_center[2];
        lens_center[0] = (eEye == Eye_Left ? left_lens_center[0] : right_lens_center[0]);
        lens_center[1] = (eEye == Eye_Left ? left_lens_center[1] : right_lens_center[1]);

        float r[2];
        r[0] = fU * viewport_scale[0] - lens_center[0];
        r[1] = fV * viewport_scale[1] - lens_center[1];

        r[0] /= warp_scale;
        r[1] /= warp_scale;

        float r_mag = sqrt(r[0] * r[0] + r[1] * r[1]);


        float r_displaced[2];
        r_displaced[0] = r[0] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[1] = r[1] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[0] *= warp_scale;
        r_displaced[1] *= warp_scale;

        float tc_r[2];
        tc_r[0] = (lens_center[0] + aberr_scale[0] * r_displaced[0]) / viewport_scale[0];
        tc_r[1] = (lens_center[1] + aberr_scale[0] * r_displaced[1]) / viewport_scale[1];

        float tc_g[2];
        tc_g[0] = (lens_center[0] + aberr_scale[1] * r_displaced[0]) / viewport_scale[0];
        tc_g[1] = (lens_center[1] + aberr_scale[1] * r_displaced[1]) / viewport_scale[1];

        float tc_b[2];
        tc_b[0] = (lens_center[0] + aberr_scale[2] * r_displaced[0]) / viewport_scale[0];
        tc_b[1] = (lens_center[1] + aberr_scale[2] * r_displaced[1]) / viewport_scale[1];

        //DriverLog("Distort %f %f -> %f %f; %f %f %f %f\n", fU, fV, tc_b[0], tc_b[1], distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3]);

        DistortionCoordinates_t coordinates;
        coordinates.rfBlue[0] = tc_b[0];
        coordinates.rfBlue[1] = tc_b[1];
        coordinates.rfGreen[0] = tc_g[0];
        coordinates.rfGreen[1] = tc_g[1];
        coordinates.rfRed[0] = tc_r[0];
        coordinates.rfRed[1] = tc_r[1];
        return coordinates;
    }

    DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        ohmd_device* d = hmdtracker ? hmdtracker : hmd;

        float quat[4];
        ohmd_device_getf(d, OHMD_ROTATION_QUAT, quat);
        pose.qRotation.x = quat[0];
        pose.qRotation.y = quat[1];
        pose.qRotation.z = quat[2];
        pose.qRotation.w = quat[3];

        float pos[3];
        ohmd_device_getf(d, OHMD_POSITION_VECTOR, pos);
        pose.vecPosition[0] = pos[0];
        pose.vecPosition[1] = pos[1];
        pose.vecPosition[2] = pos[2];

        float angvel[3];
        ohmd_device_getf(d, OHMD_ANGULAR_VELOCITY_VECTOR, angvel);
        pose.vecAngularVelocity[0] = angvel[0];
        pose.vecAngularVelocity[1] = angvel[1];
        pose.vecAngularVelocity[2] = angvel[2];

        float vel[3];
        ohmd_device_getf(d, OHMD_VELOCITY_VECTOR, vel);
        pose.vecVelocity[0] = vel[0];
        pose.vecVelocity[1] = vel[1];
        pose.vecVelocity[2] = vel[2];

        float accel[3];
        ohmd_device_getf(d, OHMD_ACCELERATION_VECTOR, accel);
        pose.vecAcceleration[0] = accel[0];
        pose.vecAcceleration[1] = accel[1];
        pose.vecAcceleration[2] = accel[2];

        // See the HMD GetPose(): report how stale the pose is so SteamVR
        // predicts forward from the sample epoch, not from "now".
        float pose_age = 0.0f;
        ohmd_device_getf(d, OHMD_POSE_AGE_SECONDS, &pose_age);
        pose.poseTimeOffset = -pose_age;

        // Everything set above is an input to SteamVR's own extrapolation, and
        // is filtered by ApplyPredictionPolicy() before this pose is returned.
        // See the policy block near identityquat for why the default keeps
        // none of it.
        //printf("%f %f %f %f  %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);
        //fflush(stdout);
        //DriverLog("get hmd pose %f %f %f %f, %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

        pose.qWorldFromDriverRotation = identityquat;
        pose.qDriverFromHeadRotation = identityquat;

        ApplyPredictionPolicy(pose);
        return pose;
    }


    void RunFrame()
    {
        // In a real driver, this should happen from some pose tracking thread.
        // The RunFrame interval is unspecified and can be very irregular if some other
        // driver blocks it for some periodic task.
        if ( m_unObjectId != vr::k_unTrackedDeviceIndexInvalid )
        {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated( m_unObjectId, GetPose(), sizeof( DriverPose_t ) );
        }
    }

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    ohmd_device* hmd = NULL;
    ohmd_device* hmdtracker = NULL;

    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sVendor;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    int32_t m_nWindowX;
    int32_t m_nWindowY;
    int32_t m_nWindowWidth;
    int32_t m_nWindowHeight;
    int32_t m_nRenderWidth;
    int32_t m_nRenderHeight;
    float m_flSecondsFromVsyncToPhotons;
    float m_flDisplayFrequency;
    float m_flIPD;
    
    float rotation_left = 0.0;
    float rotation_right = 0.0;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_OpenHMD: public IServerTrackedDeviceProvider
{
public:
    CServerDriver_OpenHMD()
        : m_OpenHMDDeviceDriver( NULL )
    {
    }
    virtual ~CServerDriver_OpenHMD() {}

    virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
    virtual void Cleanup() ;
    virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame() ;
    virtual bool ShouldBlockStandbyMode()  { return false; }
    virtual void EnterStandby()  {}
    virtual void LeaveStandby()  {}

private:
    COpenHMDDeviceDriver *m_OpenHMDDeviceDriver;
    COpenHMDDeviceDriverController *m_OpenHMDDeviceDriverControllerL;
    COpenHMDDeviceDriverController *m_OpenHMDDeviceDriverControllerR;
};

CServerDriver_OpenHMD g_serverDriverOpenHMD;

EVRInitError CServerDriver_OpenHMD::Init( vr::IVRDriverContext *pDriverContext )
{
    VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
    InitDriverLog( vr::VRDriverLog() );

    ctx = ohmd_ctx_create();
    int num_devices = ohmd_ctx_probe(ctx);
    if(num_devices < 0){
        DriverLog("failed to probe devices: %s\n", ohmd_ctx_get_error(ctx));
    }

    int hmddisplay_idx = get_configvalues()[0];
    int hmdtracker_idx = get_configvalues()[1];
    int lcontroller_idx = get_configvalues()[2];
    int rcontroller_idx = get_configvalues()[3];

    for(int i = 0; i < num_devices; i++){
        DriverLog("device %d\n", i);
        DriverLog("  vendor:  %s\n", ohmd_list_gets(ctx, i, OHMD_VENDOR));
        DriverLog("  product: %s\n", ohmd_list_gets(ctx, i, OHMD_PRODUCT));
        DriverLog("  path:    %s\n\n", ohmd_list_gets(ctx, i, OHMD_PATH));

        int device_class = 0, device_flags = 0;

        ohmd_list_geti(ctx, i, OHMD_DEVICE_CLASS, &device_class);
        ohmd_list_geti(ctx, i, OHMD_DEVICE_FLAGS, &device_flags);

	switch (device_class) {
		case OHMD_DEVICE_CLASS_HMD:
			if (hmddisplay_idx == -1)
				hmddisplay_idx = i;
			break;
		case OHMD_DEVICE_CLASS_CONTROLLER:
			if (lcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER))
				lcontroller_idx = i;
			else if (rcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER))
				rcontroller_idx = i;
			break;
		case OHMD_DEVICE_CLASS_GENERIC_TRACKER:
			if (hmdtracker_idx == -1 && !(device_flags & (OHMD_DEVICE_FLAGS_LEFT_CONTROLLER|OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER)))
				hmdtracker_idx = i;
			else if (lcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER))
				lcontroller_idx = i;
			else if (rcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER))
				rcontroller_idx = i;
			break;
		default:
			break;
	}
    }

    if (hmdtracker_idx == -1)
    	hmdtracker_idx = hmddisplay_idx;

    DriverLog("Using HMD Display %d, HMD Tracker %d, Left Controller %d, Right Controller %d\n", hmddisplay_idx, hmdtracker_idx, lcontroller_idx, rcontroller_idx);

    m_OpenHMDDeviceDriver = new COpenHMDDeviceDriver(hmddisplay_idx, hmdtracker_idx);
    vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriver->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_OpenHMDDeviceDriver );

    if (lcontroller_idx >= 0) {
	ohmd_device* lcontroller = ohmd_list_open_device(ctx, lcontroller_idx);
	if (lcontroller)
		m_OpenHMDDeviceDriverControllerL = new COpenHMDDeviceDriverController(0, lcontroller, lcontroller_idx);
	if (m_OpenHMDDeviceDriverControllerL)
		vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriverControllerL->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerL );
    }

    if (rcontroller_idx >= 0) {
	ohmd_device *rcontroller = ohmd_list_open_device(ctx, rcontroller_idx);
	if (rcontroller)
		m_OpenHMDDeviceDriverControllerR = new COpenHMDDeviceDriverController(1, rcontroller, rcontroller_idx);
	if (m_OpenHMDDeviceDriverControllerR)
		vr::VRServerDriverHost()->TrackedDeviceAdded(  m_OpenHMDDeviceDriverControllerR->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerR );
    }

    return VRInitError_None;
}

void CServerDriver_OpenHMD::Cleanup()
{
    CleanupDriverLog();
    delete m_OpenHMDDeviceDriver;
    m_OpenHMDDeviceDriver = NULL;

    if (m_OpenHMDDeviceDriverControllerL)
      delete m_OpenHMDDeviceDriverControllerL;
    if (m_OpenHMDDeviceDriverControllerR)
      delete m_OpenHMDDeviceDriverControllerR;

    if (ctx)
        ohmd_ctx_destroy (ctx);
}


void CServerDriver_OpenHMD::RunFrame()
{
    ohmd_ctx_update(ctx);

    if ( m_OpenHMDDeviceDriver )
        m_OpenHMDDeviceDriver->RunFrame();

    if (m_OpenHMDDeviceDriverControllerL)
        m_OpenHMDDeviceDriverControllerL->RunFrame();
    if (m_OpenHMDDeviceDriverControllerR)
        m_OpenHMDDeviceDriverControllerR->RunFrame();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
    if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
    {
        return &g_serverDriverOpenHMD;
    }
    if( 0 == strcmp( IVRWatchdogProvider_Version, pInterfaceName ) )
    {
        return &g_watchdogDriverOpenHMD;
    }

    DriverLog("no interface %s\n", pInterfaceName);

    if( pReturnCode )
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}
